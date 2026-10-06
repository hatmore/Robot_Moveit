#include <rclcpp/rclcpp.hpp>
#include <planning_sdk_msgs/msg/joint_velocity_once.hpp>
#include <planning_sdk_msgs/msg/joint_limits_array.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <map>
#include <vector>
#include <string>
#include <algorithm>
#include <mutex>
#include <sstream>
#include <functional>
#include <atomic>
#include <thread>

class JointVelocityToServo : public rclcpp::Node
{
public:
    explicit JointVelocityToServo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("joint_velocity_to_servo", options),
          right_arm_publisher_connected_(false),
          left_arm_publisher_connected_(false),
          joint_states_received_(false)
    {
        // 声明参数
        this->declare_parameter<bool>("enable_left_arm", true);
        this->declare_parameter<bool>("enable_right_arm", true);
        this->declare_parameter<double>("velocity_scaling_factor", 0.8);
        this->declare_parameter<bool>("verbose_logging", true);
        this->declare_parameter<double>("publish_rate", 50.0);

        // 获取参数
        bool enable_left_arm = this->get_parameter("enable_left_arm").as_bool();
        bool enable_right_arm = this->get_parameter("enable_right_arm").as_bool();
        velocity_scaling_factor_ = this->get_parameter("velocity_scaling_factor").as_double();
        verbose_logging_ = this->get_parameter("verbose_logging").as_bool();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // 参数验证
        if (velocity_scaling_factor_ <= 0 || velocity_scaling_factor_ > 1.0) {
            RCLCPP_ERROR(this->get_logger(), "速度缩放因子必须在 (0, 1.0] 范围内");
            throw std::runtime_error("无效的速度缩放因子");
        }

        initializeJointConfiguration();
        loadJointLimitsFromParams();

        last_command_hash_["left"] = 0;
        last_command_hash_["right"] = 0;
        repeated_command_count_["left"] = 0;
        repeated_command_count_["right"] = 0;

        // 关节状态接收标志
        joint_states_received_ = false;

        // 订阅动态关节限位
        auto limits_qos = rclcpp::QoS(1).transient_local();
        joint_limits_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointLimitsArray>(
            "/algorithm/joint_limits/current", limits_qos,
            std::bind(&JointVelocityToServo::jointLimitsCallback, this, std::placeholders::_1));

        // 订阅关节状态（用于位置限位）
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointVelocityToServo::jointStateCallback, this, std::placeholders::_1));

        // 创建订阅者
        joint_velocity_subscription_ = this->create_subscription<planning_sdk_msgs::msg::JointVelocityOnce>(
            "/algorithm/grasp_planning/move/joint_velocity_once",
            10,
            std::bind(&JointVelocityToServo::jointVelocityCallback, this, std::placeholders::_1));

        // 创建左臂的发布者（QoS 必须匹配 servo 节点的 SensorDataQoS 订阅）
        if (enable_left_arm) {
            auto qos = rclcpp::SensorDataQoS();
            left_arm_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(
                "/left_arm_servo_node/delta_joint_cmds",
                qos);

            RCLCPP_INFO(this->get_logger(), "左臂关节速度控制已启用");
        }

        // 创建右臂的发布者
        if (enable_right_arm) {
            auto qos = rclcpp::SensorDataQoS();
            right_arm_publisher_ = this->create_publisher<control_msgs::msg::JointJog>(
                "/right_arm_servo_node/delta_joint_cmds",
                qos);

            RCLCPP_INFO(this->get_logger(), "右臂关节速度控制已启用");
        }

        connection_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&JointVelocityToServo::checkConnections, this));

        RCLCPP_INFO(this->get_logger(), "关节速度控制节点已启动");
        RCLCPP_INFO(this->get_logger(), "速度缩放因子: %.2f", velocity_scaling_factor_);
        RCLCPP_INFO(this->get_logger(), "发布频率: %.1f Hz", publish_rate);
        RCLCPP_INFO(this->get_logger(), "详细日志: %s", verbose_logging_ ? "开启" : "关闭");

        if (!enable_left_arm && !enable_right_arm) {
            RCLCPP_WARN(this->get_logger(), "左右臂均未启用，请检查参数配置");
        }

        // 等待获取实际关节位置
        RCLCPP_INFO(this->get_logger(), "等待获取关节状态...");
        waitForJointStates();
        RCLCPP_INFO(this->get_logger(), "关节状态已获取，节点准备就绪");
    }

private:
    void initializeJointConfiguration()
    {
        left_joint_order_ = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint"
        };
        
        right_joint_order_ = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint"
        };
        
        auto initializeJoints = [this](const std::vector<std::string>& joints) {
            for (const auto& joint_name : joints) {
                joint_velocities_[joint_name] = 0.0;
                joint_max_velocities_[joint_name] = 3.0;  // 默认值，会被 loadJointLimitsFromParams 覆盖
                joint_min_positions_[joint_name] = -3.14159;
                joint_max_positions_[joint_name] = 3.14159;
                current_joint_positions_[joint_name] = 0.0;
            }
        };
        
        initializeJoints(left_joint_order_);
        initializeJoints(right_joint_order_);

        joint_init_velocities_ = joint_velocities_;
    }

    void loadJointLimitsFromParams()
    {
        for (auto& [joint_name, max_vel] : joint_max_velocities_) {
            std::string param_name = "robot_description_planning.joint_limits." + joint_name + ".max_velocity";
            this->declare_parameter<double>(param_name, max_vel);
            double val = this->get_parameter(param_name).as_double();
            if (val > 0.0) {
                max_vel = val;
            }
            RCLCPP_INFO(this->get_logger(), "关节[%s] max_velocity=%.2f", joint_name.c_str(), max_vel);
        }
    }

    void jointLimitsCallback(const planning_sdk_msgs::msg::JointLimitsArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (const auto& jl : msg->limits) {
            auto it = joint_max_velocities_.find(jl.joint_name);
            if (it != joint_max_velocities_.end() && jl.velocity_limit > 0.0) {
                it->second = jl.velocity_limit;
            }
            auto it_min = joint_min_positions_.find(jl.joint_name);
            if (it_min != joint_min_positions_.end()) {
                it_min->second = jl.lower_limit;
                joint_max_positions_[jl.joint_name] = jl.upper_limit;
            }
        }
        RCLCPP_INFO(this->get_logger(), "Updated max velocities from joint limits (%zu joints)", msg->limits.size());
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        bool has_valid_data = false;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = current_joint_positions_.find(msg->name[i]);
            if (it != current_joint_positions_.end()) {
                it->second = msg->position[i];
                has_valid_data = true;
            }
        }
        // 标记已收到关节状态（至少包含一个我们关心的关节）
        if (has_valid_data) {
            joint_states_received_ = true;
        }
    }

    // 等待获取关节状态
    void waitForJointStates(double timeout_sec = 5.0) {
        auto start_time = this->now();
        rclcpp::Rate rate(10);  // 10Hz 检查

        while (rclcpp::ok()) {
            // 检查是否已收到关节状态
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                if (joint_states_received_) {
                    return;
                }
            }

            // 超时检查
            auto elapsed = (this->now() - start_time).seconds();
            if (elapsed > timeout_sec) {
                RCLCPP_WARN(this->get_logger(), "等待关节状态超时 (%.1f秒)，将继续使用默认值", timeout_sec);
                // 使用默认值继续
                joint_states_received_ = true;
                return;
            }

            // 处理回调
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

    // 软限位硬截断：到达限位时屏蔽超限方向速度，允许返回限位内方向速度
    double clampVelocityForPosition(const std::string& joint_name, double velocity) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        auto it = current_joint_positions_.find(joint_name);
        if (it == current_joint_positions_.end()) return velocity;
        double pos = it->second;
        double min_pos = joint_min_positions_[joint_name];
        double max_pos = joint_max_positions_[joint_name];

        // 已达到或超过上限，且速度方向朝向超限 → 屏蔽；反向（回限位内）→ 允许
        if (velocity > 0.0 && pos >= max_pos) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "关节[%s] 已到达上限位 (pos=%.4f >= max=%.4f)，屏蔽正向速度",
                joint_name.c_str(), pos, max_pos);
            return 0.0;
        }
        // 已达到或超过下限，且速度方向朝向超限 → 屏蔽；反向（回限位内）→ 允许
        if (velocity < 0.0 && pos <= min_pos) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "关节[%s] 已到达下限位 (pos=%.4f <= min=%.4f)，屏蔽负向速度",
                joint_name.c_str(), pos, min_pos);
            return 0.0;
        }
        return velocity;
    }

    void checkConnections()
    {
        auto check_publisher_connection = [this](const std::string& arm_side,
                                                const rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr& publisher,
                                                bool& connected_flag) {
            if (publisher) {
                auto subscribers = publisher->get_subscription_count();
                if (subscribers > 0 && !connected_flag) {
                    connected_flag = true;
                    RCLCPP_INFO(this->get_logger(), "%s臂Servo节点已连接 (订阅者数量: %zu)",
                               arm_side.c_str(), subscribers);
                } else if (subscribers == 0 && connected_flag) {
                    connected_flag = false;
                    RCLCPP_WARN(this->get_logger(), "%s臂Servo节点连接丢失", arm_side.c_str());
                }
            }
        };

        check_publisher_connection("左", left_arm_publisher_, left_arm_publisher_connected_);
        check_publisher_connection("右", right_arm_publisher_, right_arm_publisher_connected_);
    }
    
    void jointVelocityCallback(const planning_sdk_msgs::msg::JointVelocityOnce::SharedPtr msg)
    {
        try
        {
            // 检查是否已收到有效的关节状态
            if (!joint_states_received_) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "尚未收到有效关节状态，忽略速度命令");
                return;
            }

            joint_velocities_ = joint_init_velocities_;
            std::string joint_name = msg->joint_name;
            int16_t velocity_percent = msg->velocity;
            
            std::string arm_side = "unknown";
            if (joint_name.find("left") != std::string::npos) {
                arm_side = "left";
                if (!left_arm_publisher_connected_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                       "左臂Servo节点未连接，忽略速度命令");
                    return;
                }
            } else if (joint_name.find("right") != std::string::npos) {
                arm_side = "right";
                if (!right_arm_publisher_connected_) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                       "右臂Servo节点未连接，忽略速度命令");
                    return;
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "未知的关节名称: %s，无法判断属于左臂还是右臂", joint_name.c_str());
                return;
            }

            if (joint_velocities_.find(joint_name) == joint_velocities_.end()) {
                RCLCPP_WARN(this->get_logger(), "未知关节名称: %s", joint_name.c_str());
                return;
            }
            
            if (velocity_percent < -100 || velocity_percent > 100) {
                RCLCPP_WARN(this->get_logger(), "速度百分比超出范围 [-100, 100]: %d", velocity_percent);
                return;
            }
            
            double joint_max_velocity = joint_max_velocities_[joint_name];
            double scaled_max_velocity = joint_max_velocity * velocity_scaling_factor_;
            double actual_velocity = (static_cast<double>(velocity_percent) / 100.0) * scaled_max_velocity;

            actual_velocity = clampVelocityForPosition(joint_name, actual_velocity);
            joint_velocities_[joint_name] = actual_velocity;

            // 只发布到对应的臂，避免左右臂干扰
            if (arm_side == "left" && left_arm_publisher_connected_ && left_arm_publisher_) {
                auto left_arm_msg = std::make_unique<control_msgs::msg::JointJog>();
                left_arm_msg->header.stamp = this->now();
                left_arm_msg->header.frame_id = "base_link";

                for (const auto& jname : left_joint_order_) {
                    left_arm_msg->joint_names.push_back(jname);
                    left_arm_msg->velocities.push_back(joint_velocities_[jname]);
                }
                left_arm_msg->duration = 0.02;  // 2x publish_period(0.01s), gives servo lookahead buffer
                left_arm_publisher_->publish(std::move(left_arm_msg));
            } else if (arm_side == "right" && right_arm_publisher_connected_ && right_arm_publisher_) {
                auto right_arm_msg = std::make_unique<control_msgs::msg::JointJog>();
                right_arm_msg->header.stamp = this->now();
                right_arm_msg->header.frame_id = "base_link";

                for (const auto& jname : right_joint_order_) {
                    right_arm_msg->joint_names.push_back(jname);
                    right_arm_msg->velocities.push_back(joint_velocities_[jname]);
                }
                right_arm_msg->duration = 0.02;  // 2x publish_period(0.01s), gives servo lookahead buffer
                right_arm_publisher_->publish(std::move(right_arm_msg));
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "处理关节速度命令时发生错误: %s", e.what());
        }
    }
    
private:

    double velocity_scaling_factor_;
    bool verbose_logging_;

    bool right_arm_publisher_connected_;
    bool left_arm_publisher_connected_;

    // 关节状态接收标志
    std::atomic<bool> joint_states_received_;

    std::map<std::string, double> joint_velocities_;
    std::map<std::string, double> joint_init_velocities_;
    std::map<std::string, double> joint_max_velocities_;
    std::map<std::string, double> joint_min_positions_;
    std::map<std::string, double> joint_max_positions_;
    std::map<std::string, double> current_joint_positions_;
    std::mutex joint_state_mutex_;
    std::vector<std::string> left_joint_order_;
    std::vector<std::string> right_joint_order_;          
    
    std::map<std::string, size_t> last_command_hash_;
    std::map<std::string, int> repeated_command_count_;

    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr connection_check_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_publisher_;
    rclcpp::Subscription<planning_sdk_msgs::msg::JointVelocityOnce>::SharedPtr joint_velocity_subscription_;
    rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr left_arm_publisher_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr right_arm_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "启动关节速度控制节点");
    auto node = std::make_shared<JointVelocityToServo>();
    RCLCPP_INFO(node->get_logger(), "节点已启动，等待关节速度控制命令");
    rclcpp::spin(node);
    RCLCPP_INFO(rclcpp::get_logger("main"), "关闭关节速度控制节点");
    rclcpp::shutdown();
    return 0;
}