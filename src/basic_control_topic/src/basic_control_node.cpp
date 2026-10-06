/**
 * ================================================================================
 * Basic Control Node - 统一入口点
 * ================================================================================
 *
 * 将所有 basic_control_topic 的功能合并到单一进程中：
 *   1. PositionServersNode - 位置控制（关节位置、TCP位置、增量控制）
 *   2. VelocityControllersNode - 速度控制（TCP速度、关节速度）
 *   3. 其他服务和发布节点
 *
 * 优势:
 *   - 减少进程数量（从15个减少到1个）
 *   - 共享资源（关节限位、关节状态等）
 *   - 降低内存占用
 *
 * ================================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "basic_control_topic/shared_resources.hpp"
#include "basic_control_topic/components/position_servers_node.hpp"
#include "basic_control_topic/components/velocity_controllers_node.hpp"

#include <planning_sdk_msgs/msg/joint_limits_array.hpp>
#include <planning_sdk_msgs/msg/tcp_pose.hpp>
#include <planning_sdk_msgs/msg/heart_beat.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <cerebellum_sdk_msg/msg/motor_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <planning_sdk_msgs/srv/register_frame.hpp>
#include <planning_sdk_msgs/srv/set_joint_limits.hpp>
#include <planning_sdk_msgs/srv/get_joint_limits.hpp>

#include <urdf/model.h>
#include <fstream>
#include <sstream>
#include <cstdlib>

// ============================================================================
// 辅助节点：关节限位服务器
// ============================================================================
class JointLimitsServerNode : public rclcpp::Node
{
public:
    JointLimitsServerNode(const basic_control_topic::SharedResources::Ptr& shared_resources)
        : Node("joint_limits_server"), shared_resources_(shared_resources)
    {
        this->declare_parameter<bool>("load_persisted_on_startup", true);
        this->declare_parameter<std::string>("persist_file_path", getDefaultPersistPath());

        initJointNames();
        loadDefaultsFromURDF();
        loadDefaultsFromParams();

        if (this->get_parameter("load_persisted_on_startup").as_bool())
        {
            loadPersistedOverrides();
        }

        auto qos = rclcpp::QoS(1).transient_local();
        limits_pub_ = this->create_publisher<planning_sdk_msgs::msg::JointLimitsArray>(
            "/algorithm/joint_limits/current", qos);

        set_srv_ = this->create_service<planning_sdk_msgs::srv::SetJointLimits>(
            "/algorithm/grasp_planning/set_joint_limits",
            std::bind(&JointLimitsServerNode::handleSet, this, std::placeholders::_1, std::placeholders::_2));

        get_srv_ = this->create_service<planning_sdk_msgs::srv::GetJointLimits>(
            "/algorithm/grasp_planning/get_joint_limits",
            std::bind(&JointLimitsServerNode::handleGet, this, std::placeholders::_1, std::placeholders::_2));

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cerebellum_sdk/arm/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                current_joint_state_ = *msg;
            });

        publishCurrentLimits();
        RCLCPP_INFO(this->get_logger(), "Joint limits server started with %zu joints", limits_.size());
    }

private:
    basic_control_topic::SharedResources::Ptr shared_resources_;

    struct JointLimitData {
        double min_position = -3.14159;
        double max_position = 3.14159;
        double max_velocity = 3.0;
        double effort_limit = 0.0;
    };

    std::vector<std::string> joint_names_;
    std::map<std::string, JointLimitData> limits_;
    std::map<std::string, std::pair<double, double>> urdf_hard_limits_;
    std::mutex mutex_;

    rclcpp::Publisher<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr limits_pub_;
    rclcpp::Service<planning_sdk_msgs::srv::SetJointLimits>::SharedPtr set_srv_;
    rclcpp::Service<planning_sdk_msgs::srv::GetJointLimits>::SharedPtr get_srv_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    sensor_msgs::msg::JointState current_joint_state_;
    std::mutex joint_state_mutex_;

    std::string getDefaultPersistPath()
    {
        return "/ros2_ws/linden_robot_moveit/src/teleop_description_moveit_config/config/joint_limits_override.yaml";
    }

    void initJointNames()
    {
        joint_names_ = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint"
        };
        for (const auto& name : joint_names_)
        {
            limits_[name] = JointLimitData{};
        }
    }

    void loadDefaultsFromURDF()
    {
        this->declare_parameter<std::string>("robot_description", "");
        std::string urdf_string = this->get_parameter("robot_description").as_string();
        if (urdf_string.empty())
        {
            RCLCPP_WARN(this->get_logger(), "No robot_description parameter");
            return;
        }

        urdf::Model model;
        if (!model.initString(urdf_string))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return;
        }

        for (const auto& name : joint_names_)
        {
            auto joint = model.getJoint(name);
            if (joint && joint->limits)
            {
                auto& lim = limits_[name];
                lim.min_position = joint->limits->lower;
                lim.max_position = joint->limits->upper;
                lim.max_velocity = joint->limits->velocity;
                lim.effort_limit = joint->limits->effort;
                urdf_hard_limits_[name] = {joint->limits->lower, joint->limits->upper};
            }
        }
    }

    void loadDefaultsFromParams()
    {
        for (const auto& name : joint_names_)
        {
            auto& lim = limits_[name];
            std::string prefix = "robot_description_planning.joint_limits." + name;
            auto declare_if_needed = [this](const std::string& p, double def) -> double {
                if (!this->has_parameter(p))
                {
                    this->declare_parameter<double>(p, def);
                }
                return this->get_parameter(p).as_double();
            };
            lim.max_velocity = declare_if_needed(prefix + ".max_velocity", lim.max_velocity);
            lim.min_position = declare_if_needed(prefix + ".min_position", lim.min_position);
            lim.max_position = declare_if_needed(prefix + ".max_position", lim.max_position);
        }
    }

    void loadPersistedOverrides()
    {
        std::string persist_path = this->get_parameter("persist_file_path").as_string();
        std::ifstream file(persist_path);
        if (!file.is_open()) return;

        std::string line, current_joint;
        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#') continue;
            if (line.back() == ':' && line.find(' ') == std::string::npos)
            {
                current_joint = line.substr(0, line.size() - 1);
                continue;
            }
            if (current_joint.empty() || limits_.find(current_joint) == limits_.end()) continue;

            std::istringstream iss(line);
            std::string key;
            double val;
            iss >> key >> val;
            if (key.empty() || key.back() != ':') continue;
            key.pop_back();

            auto& lim = limits_[current_joint];
            if (key == "min_position") lim.min_position = val;
            else if (key == "max_position") lim.max_position = val;
            else if (key == "max_velocity") lim.max_velocity = val;
            else if (key == "effort_limit") lim.effort_limit = val;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded persisted overrides from %s", persist_path.c_str());
    }

    void persistToFile()
    {
        std::string persist_path = this->get_parameter("persist_file_path").as_string();
        auto dir = persist_path.substr(0, persist_path.rfind('/'));
        if (!dir.empty())
        {
            std::string cmd = "mkdir -p " + dir;
            (void)system(cmd.c_str());
        }

        std::ofstream file(persist_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to write persist file: %s", persist_path.c_str());
            return;
        }

        file << "# Joint limits overrides (auto-generated)\n";
        for (const auto& [name, lim] : limits_)
        {
            file << name << ":\n";
            file << "  min_position: " << lim.min_position << "\n";
            file << "  max_position: " << lim.max_position << "\n";
            file << "  max_velocity: " << lim.max_velocity << "\n";
            file << "  effort_limit: " << lim.effort_limit << "\n";
        }

        RCLCPP_INFO(this->get_logger(), "Persisted limits to %s", persist_path.c_str());
    }

    void publishCurrentLimits()
    {
        auto msg = planning_sdk_msgs::msg::JointLimitsArray();
        msg.timestamp = this->now();
        for (const auto& [name, lim] : limits_)
        {
            planning_sdk_msgs::msg::JointLimit jl;
            jl.joint_name = name;
            jl.lower_limit = lim.min_position;
            jl.upper_limit = lim.max_position;
            jl.velocity_limit = lim.max_velocity;
            jl.effort_limit = lim.effort_limit;
            msg.limits.push_back(jl);
        }
        limits_pub_->publish(msg);
    }

    void handleSet(
        const planning_sdk_msgs::srv::SetJointLimits::Request::SharedPtr req,
        planning_sdk_msgs::srv::SetJointLimits::Response::SharedPtr res)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        res->timestamp = this->now();

        if (req->limits.empty())
        {
            res->error_code = 3;
            res->error_message = "limits array is empty";
            return;
        }

        for (const auto& jl : req->limits)
        {
            auto it = limits_.find(jl.joint_name);
            if (it == limits_.end())
            {
                res->error_code = 1;
                res->error_message = "Unknown joint: " + jl.joint_name;
                RCLCPP_ERROR(this->get_logger(), "REJECT: %s", res->error_message.c_str());
                return;
            }

            auto hard_it = urdf_hard_limits_.find(jl.joint_name);
            if (hard_it != urdf_hard_limits_.end())
            {
                if (jl.lower_limit < hard_it->second.first)
                {
                    res->error_code = 2;
                    res->error_message = jl.joint_name + " lower_limit " +
                        std::to_string(jl.lower_limit) + " < URDF hard min " + std::to_string(hard_it->second.first);
                    RCLCPP_ERROR(this->get_logger(), "REJECT: %s", res->error_message.c_str());
                    return;
                }
                if (jl.upper_limit > hard_it->second.second)
                {
                    res->error_code = 2;
                    res->error_message = jl.joint_name + " upper_limit " +
                        std::to_string(jl.upper_limit) + " > URDF hard max " + std::to_string(hard_it->second.second);
                    RCLCPP_ERROR(this->get_logger(), "REJECT: %s", res->error_message.c_str());
                    return;
                }
            }

            // 检查当前关节位置不超出新限位
            {
                std::lock_guard<std::mutex> state_lock(joint_state_mutex_);
                for (size_t i = 0; i < current_joint_state_.name.size(); ++i)
                {
                    if (current_joint_state_.name[i] == jl.joint_name)
                    {
                        double pos = current_joint_state_.position[i];
                        if (pos < jl.lower_limit || pos > jl.upper_limit)
                        {
                            res->error_code = 2;
                            res->error_message = jl.joint_name + " current pos " +
                                std::to_string(pos) + " exceeds new limits [" +
                                std::to_string(jl.lower_limit) + ", " + std::to_string(jl.upper_limit) + "]";
                            RCLCPP_ERROR(this->get_logger(), "REJECT: %s", res->error_message.c_str());
                            return;
                        }
                        break;
                    }
                }
            }
        }

        for (const auto& jl : req->limits)
        {
            auto& lim = limits_[jl.joint_name];
            lim.min_position = jl.lower_limit;
            lim.max_position = jl.upper_limit;
            lim.max_velocity = jl.velocity_limit;
            lim.effort_limit = jl.effort_limit;
            RCLCPP_INFO(this->get_logger(), "Set [%s] pos=[%.3f,%.3f] vel=%.3f effort=%.3f",
                        jl.joint_name.c_str(), lim.min_position, lim.max_position,
                        lim.max_velocity, lim.effort_limit);
        }

        persistToFile();
        publishCurrentLimits();
        res->error_code = 0;
        res->error_message = "OK";
    }

    void handleGet(
        const planning_sdk_msgs::srv::GetJointLimits::Request::SharedPtr req,
        planning_sdk_msgs::srv::GetJointLimits::Response::SharedPtr res)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        res->timestamp = this->now();

        auto toMsg = [this](const std::string& name, const JointLimitData& lim) {
            planning_sdk_msgs::msg::JointLimit msg;
            msg.joint_name = name;
            msg.lower_limit = lim.min_position;
            msg.upper_limit = lim.max_position;
            msg.velocity_limit = lim.max_velocity;
            msg.effort_limit = lim.effort_limit;
            return msg;
        };

        if (req->joint_names.empty())
        {
            for (const auto& [name, lim] : limits_)
            {
                res->limits.push_back(toMsg(name, lim));
            }
            res->error_code = 0;
            res->error_message = "Returned all " + std::to_string(limits_.size()) + " joints";
        }
        else
        {
            for (const auto& name : req->joint_names)
            {
                auto it = limits_.find(name);
                if (it == limits_.end())
                {
                    res->error_code = 1;
                    res->error_message = "Unknown joint: " + name;
                    return;
                }
                res->limits.push_back(toMsg(it->first, it->second));
            }
            res->error_code = 0;
            res->error_message = "OK";
        }
    }
};

// ============================================================================
// 辅助节点：TCP位姿发布
// ============================================================================
class TcpPosePublisherNode : public rclcpp::Node
{
public:
    TcpPosePublisherNode() : Node("tcp_pose_publisher")
    {
        this->declare_parameter<std::string>("left_source_frame", "left_gripper_base_link");
        this->declare_parameter<std::string>("right_source_frame", "right_gripper_base_link");
        this->declare_parameter<std::string>("target_frame", "base_link");

        left_source_frame_ = this->get_parameter("left_source_frame").as_string();
        right_source_frame_ = this->get_parameter("right_source_frame").as_string();
        target_frame_ = this->get_parameter("target_frame").as_string();

        left_publisher_ = this->create_publisher<planning_sdk_msgs::msg::TcpPose>(
            "/algorithm/grasp_planning/left_arm/tcp_pose", 10);
        right_publisher_ = this->create_publisher<planning_sdk_msgs::msg::TcpPose>(
            "/algorithm/grasp_planning/right_arm/tcp_pose", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&TcpPosePublisherNode::publishTcpPose, this));

        RCLCPP_INFO(this->get_logger(), "TcpPose发布节点已启动！");
        RCLCPP_INFO(this->get_logger(), "当前配置：");
        RCLCPP_INFO(this->get_logger(), "  目标坐标系：%s", target_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  左臂源帧：%s", left_source_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  右臂源帧：%s", right_source_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "  发布频率：1.00 Hz");
        RCLCPP_INFO(this->get_logger(), "  左臂话题：/algorithm/grasp_planning/left_arm/tcp_pose");
        RCLCPP_INFO(this->get_logger(), "  右臂话题：/algorithm/grasp_planning/right_arm/tcp_pose");
    }

private:
    void publishTcpPose()
    {
        auto get_and_publish = [this](
            const std::string& source_frame,
            rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr publisher,
            const std::string& arm_name) {

            auto msg = planning_sdk_msgs::msg::TcpPose();
            msg.error_code = 0;
            msg.timestamp = this->get_clock()->now();

            try
            {
                auto transform = tf_buffer_->lookupTransform(
                    target_frame_, source_frame, tf2::TimePointZero, std::chrono::milliseconds(500));

                geometry_msgs::msg::Pose pose;
                pose.position.x = transform.transform.translation.x;
                pose.position.y = transform.transform.translation.y;
                pose.position.z = transform.transform.translation.z;
                pose.orientation = transform.transform.rotation;
                msg.current_pose = pose;
            }
            catch (const tf2::TransformException& ex)
            {
                RCLCPP_WARN(this->get_logger(), "%s TF查询失败: %s", arm_name.c_str(), ex.what());
                msg.error_code = 1;
            }

            publisher->publish(msg);
        };

        get_and_publish(left_source_frame_, left_publisher_, "左臂");
        get_and_publish(right_source_frame_, right_publisher_, "右臂");
    }

    rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr left_publisher_;
    rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr right_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::string left_source_frame_;
    std::string right_source_frame_;
    std::string target_frame_;
};

// ============================================================================
// 辅助节点：心跳
// ============================================================================
// 左臂 Servo 错误码基址
static constexpr uint16_t LEFT_BASE  = 0x0010;
// 右臂 Servo 错误码基址
static constexpr uint16_t RIGHT_BASE = 0x0020;
// Servo 连接超时（秒）
static constexpr double SERVO_DISCONNECT_TIMEOUT_SEC = 5.0;

// Action Server 存活检测
struct NodeCheck {
    const char* action_status_topic;
    uint16_t    error_code;
    const char* error_msg;
};

static const NodeCheck NODE_CHECKS[] = {
    {"/algorithm/grasp_planning/move/joint_position/_action/status", 0x0101, "joint_position_server 不可用"},
    {"/algorithm/grasp_planning/move/joint_position_delta/_action/status", 0x0102, "joint_position_delta_server 不可用"},
    {"/algorithm/grasp_planning/move/left_arm/execute_trajectory/_action/status", 0x0103, "execute_trajectory_server 不可用"},
    {"/algorithm/grasp_planning/move/left_arm/tcp_position/_action/status", 0x0104, "tcp_position_action_server 不可用"},
    {"/algorithm/grasp_planning/move/left_arm/tcp_position_delta/_action/status", 0x0105, "tcp_position_delta_action_server 不可用"},
    {"/algorithm/grasp_planning/move/left_arm/tcp_trajectory/_action/status", 0x0106, "tcp_trajectory_action_server 不可用"},
    {"/algorithm/joint_limits/current", 0x0107, "joint_limits_server 不可用"},
};

class HeartBeatNode : public rclcpp::Node
{
public:
    HeartBeatNode() : Node("heart_beat")
    {
        left_status_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/left_arm_servo_node/status", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                left_servo_status_ = msg->data;
                left_last_recv_ = now();
            });

        right_status_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/right_arm_servo_node/status", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(mutex_);
                right_servo_status_ = msg->data;
                right_last_recv_ = now();
            });

        publisher_ = create_publisher<planning_sdk_msgs::msg::HeartBeat>(
            "/algorithm/grasp_planning/heart_beat", 10);

        publish_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HeartBeatNode::publishHeartBeat, this));

        RCLCPP_INFO(get_logger(), "HeartBeat 节点已启动，发布频率 1Hz");
        RCLCPP_INFO(get_logger(), "监控 Servo 状态话题: /left_arm_servo_node/status, /right_arm_servo_node/status");
        RCLCPP_INFO(get_logger(), "监控 Action Server 节点: %zu 个", sizeof(NODE_CHECKS) / sizeof(NODE_CHECKS[0]));
    }

private:
    void publishHeartBeat()
    {
        auto msg = planning_sdk_msgs::msg::HeartBeat();
        auto t = now();

        // Servo 状态
        {
            std::lock_guard<std::mutex> lock(mutex_);

            // 连接超时检测
            if (isTimedOut(left_last_recv_, t)) {
                addError(msg, 0x0001, "左臂Servo节点无响应");
            }
            if (isTimedOut(right_last_recv_, t)) {
                addError(msg, 0x0002, "右臂Servo节点无响应");
            }

            // Servo 运行状态错误
            collectServoErrors(msg, left_servo_status_, LEFT_BASE, "左");
            collectServoErrors(msg, right_servo_status_, RIGHT_BASE, "右");
        }

        // Action Server 存活检测
        for (const auto& check : NODE_CHECKS) {
            if (count_publishers(check.action_status_topic) == 0) {
                addError(msg, check.error_code, check.error_msg);
            }
        }

        publisher_->publish(msg);
    }

    bool isTimedOut(const rclcpp::Time& last_recv, const rclcpp::Time& t) const
    {
        if (last_recv.nanoseconds() == 0) {
            return false;
        }
        return (t - last_recv).seconds() > SERVO_DISCONNECT_TIMEOUT_SEC;
    }

    void collectServoErrors(planning_sdk_msgs::msg::HeartBeat& msg,
                            int8_t status, uint16_t base, const std::string& arm_name)
    {
        if (status == 0) return;

        static const char* const servo_status_text[] = {
            "", "Servo: 接近奇异点，正在减速", "Servo: 非常接近奇异点，硬停止",
            "Servo: 接近碰撞，正在减速", "Servo: 检测到碰撞，硬停止",
            "Servo: 接近关节限位，停止", "Servo: 正在离开奇异点，减速中",
        };

        if (status >= 1 && status <= 6) {
            addError(msg, static_cast<uint16_t>(base + status), arm_name + servo_status_text[status]);
        } else {
            addError(msg, static_cast<uint16_t>(base + 0x0F), arm_name + "Servo: 未知状态码 " + std::to_string(status));
        }
    }

    void addError(planning_sdk_msgs::msg::HeartBeat& msg, uint16_t code, const std::string& text)
    {
        msg.error_code.push_back(code);
        msg.error_msg.push_back(text);
    }

    std::mutex mutex_;
    int8_t left_servo_status_{0};
    int8_t right_servo_status_{0};
    rclcpp::Time left_last_recv_{0, 0, RCL_ROS_TIME};
    rclcpp::Time right_last_recv_{0, 0, RCL_ROS_TIME};
    rclcpp::Publisher<planning_sdk_msgs::msg::HeartBeat>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr left_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr right_status_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

// ============================================================================
// 辅助节点：Servo管理
// ============================================================================
class ServoManagerNode : public rclcpp::Node
{
public:
    ServoManagerNode(const basic_control_topic::SharedResources::Ptr& shared_resources = nullptr)
        : Node("servo_manager"), shared_resources_(shared_resources), is_servo_running_(false), is_starting_(false)
    {
        servo_joint_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointVelocityOnce>(
            "/algorithm/grasp_planning/move/joint_velocity_once", 10,
            [this](const planning_sdk_msgs::msg::JointVelocityOnce::SharedPtr) {
                updateLastTime();
                requestStartServo();
            });

        servo_left_tcp_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
            "/algorithm/grasp_planning/move/left_arm/tcp_velocity_once", 10,
            [this](const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr) {
                updateLastTime();
                requestStartServo();
            });

        servo_right_tcp_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
            "/algorithm/grasp_planning/move/right_arm/tcp_velocity_once", 10,
            [this](const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr) {
                updateLastTime();
                requestStartServo();
            });

        // 创建单独的节点用于服务调用的 spin
        spin_node_ = std::make_shared<rclcpp::Node>("servo_manager_spin_node");
        left_start_cli_ = spin_node_->create_client<std_srvs::srv::Trigger>("/left_arm_servo_node/start_servo");
        left_stop_cli_ = spin_node_->create_client<std_srvs::srv::Trigger>("/left_arm_servo_node/stop_servo");
        right_start_cli_ = spin_node_->create_client<std_srvs::srv::Trigger>("/right_arm_servo_node/start_servo");
        right_stop_cli_ = spin_node_->create_client<std_srvs::srv::Trigger>("/right_arm_servo_node/stop_servo");

        // 定时检查是否需要停止 Servo
        check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ServoManagerNode::checkCallback, this));

        // 注册急停回调，立即停止 servo
        if (shared_resources_) {
            shared_resources_->registerEmergencyStopCallback([this]() {
                RCLCPP_WARN(this->get_logger(), "Emergency stop: stopping servo immediately");
                if (is_servo_running_) {
                    std::thread([this]() { stopServo(); }).detach();
                }
            });
        }

        RCLCPP_INFO(this->get_logger(), "Servo Manager 已启动");
    }

private:
    void updateLastTime()
    {
        std::lock_guard<std::mutex> lock(last_time_mutex_);
        last_time_ = this->get_clock()->now();
    }

    void requestStartServo()
    {
        if (is_servo_running_ || is_starting_) return;
        is_starting_ = true;
        std::thread([this]() {
            startServo();
            is_starting_ = false;
        }).detach();
    }

    void checkCallback()
    {
        if (is_servo_running_)
        {
            std::lock_guard<std::mutex> lock(last_time_mutex_);
            if ((this->get_clock()->now() - last_time_).nanoseconds() > 1000000000)
            {
                std::thread([this]() { stopServo(); }).detach();
            }
        }
    }

    void startServo()
    {
        if (!left_start_cli_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "Left arm servo start service not available");
            return;
        }

        if (!right_start_cli_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "Right arm servo start service not available");
            return;
        }

        // 左右臂并行发送启动请求
        auto left_future = left_start_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        auto right_future = right_start_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());

        bool left_ok = false, right_ok = false;

        if (rclcpp::spin_until_future_complete(spin_node_, left_future, std::chrono::seconds(3)) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto resp = left_future.get();
            left_ok = resp->success;
            if (!left_ok) {
                RCLCPP_ERROR(this->get_logger(), "Left arm servo start failed: %s", resp->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Left arm servo start service call timeout");
        }

        if (rclcpp::spin_until_future_complete(spin_node_, right_future, std::chrono::seconds(3)) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto resp = right_future.get();
            right_ok = resp->success;
            if (!right_ok) {
                RCLCPP_ERROR(this->get_logger(), "Right arm servo start failed: %s", resp->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Right arm servo start service call timeout");
        }

        if (left_ok && right_ok) {
            RCLCPP_INFO(this->get_logger(), "servo auto start !!");
            is_servo_running_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "servo start partial: left=%s, right=%s",
                        left_ok ? "ok" : "failed", right_ok ? "ok" : "failed");
            is_servo_running_ = true;
        }
    }

    void stopServo()
    {
        if (left_stop_cli_->service_is_ready()) {
            left_stop_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        }
        if (right_stop_cli_->service_is_ready()) {
            right_stop_cli_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
        }

        RCLCPP_INFO(this->get_logger(), "servo auto stop !!");
        is_servo_running_ = false;
    }

    basic_control_topic::SharedResources::Ptr shared_resources_;
    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr servo_left_tcp_sub_;
    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr servo_right_tcp_sub_;
    rclcpp::Subscription<planning_sdk_msgs::msg::JointVelocityOnce>::SharedPtr servo_joint_sub_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    std::mutex last_time_mutex_;
    rclcpp::Time last_time_;

    std::atomic<bool> is_servo_running_;
    std::atomic<bool> is_starting_;

    rclcpp::Node::SharedPtr spin_node_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_stop_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_start_cli_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_stop_cli_;
};

// ============================================================================
// 辅助节点：仿真接口切换
// ============================================================================
// URDF 硬限位，用于归一化实机多圈旋转导致的超限关节值
static const std::unordered_map<std::string, std::pair<double, double>> URDF_LIMITS = {
    {"left_shoulder_pitch_joint",  {-4.36332,   1.22173}},
    {"left_shoulder_roll_joint",   {-0.733038,  2.879793}},
    {"left_shoulder_yaw_joint",    {-2.7925268, 2.7925268}},
    {"left_elbow_joint",           {-2.338741,  2.338741}},
    {"left_wrist_roll_joint",      {-2.8448867, 2.8448867}},
    {"left_wrist_yaw_joint",       {-0.959931,  0.959931}},
    {"left_wrist_pitch_joint",     {-1.570796,  1.570796}},
    {"right_shoulder_pitch_joint", {-1.22173,   4.36332}},
    {"right_shoulder_roll_joint",  {-2.879793,  0.733038}},
    {"right_shoulder_yaw_joint",   {-2.7925268, 2.7925268}},
    {"right_elbow_joint",          {-2.338741,  2.338741}},
    {"right_wrist_roll_joint",     {-2.8448867, 2.8448867}},
    {"right_wrist_yaw_joint",      {-0.959931,  0.959931}},
    {"right_wrist_pitch_joint",    {-1.570796,  1.570796}},
};

static double normalizeJointPosition(const std::string& name, double pos)
{
    auto it = URDF_LIMITS.find(name);
    if (it == URDF_LIMITS.end()) return pos;
    double lo = it->second.first;
    double hi = it->second.second;
    while (pos < lo) pos += 2.0 * M_PI;
    while (pos > hi) pos -= 2.0 * M_PI;
    if (pos < lo || pos > hi) {
        pos = std::max(lo, std::min(hi, pos));
    }
    return pos;
}

class SimInterfaceSwitchNode : public rclcpp::Node
{
public:
    SimInterfaceSwitchNode(const basic_control_topic::SharedResources::Ptr& shared_resources = nullptr)
        : Node("sim_interface_switch_node"), shared_resources_(shared_resources), use_simulator_(false)
    {
        switch_service_ = this->create_service<std_srvs::srv::SetBool>(
            "/algorithm/grasp_planning/simulator/mode",
            std::bind(&SimInterfaceSwitchNode::handleSwitchRequest, this, std::placeholders::_1, std::placeholders::_2));

        // 控制器切换客户端（用于模式切换时安全重启控制器，防止飞车）
        switch_controller_cli_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
        real_joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/cerebellum_sdk/arm/joint_commands", 10);
        sim_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/algorithm/grasp_planning/simulator/joint_state", 10);

        sim_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/sim_joint_states", 10,
            std::bind(&SimInterfaceSwitchNode::simJointStateCallback, this, std::placeholders::_1));

        joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_commands", 10,
            std::bind(&SimInterfaceSwitchNode::jointCommandCallback, this, std::placeholders::_1));

        real_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cerebellum_sdk/arm/joint_states", rclcpp::SensorDataQoS(),
            std::bind(&SimInterfaceSwitchNode::realJointStateCallback, this, std::placeholders::_1));

        // 订阅电机状态（急停检测）
        motor_states_sub_ = this->create_subscription<cerebellum_sdk_msg::msg::MotorState>(
            "/cerebellum_sdk/arm/motor_states", 10,
            [this](const cerebellum_sdk_msg::msg::MotorState::SharedPtr msg) {
                uint8_t mode = (msg->run_mode.size() > 0) ? msg->run_mode[0] : 0;
                int prev = current_run_mode_.load();
                current_run_mode_.store(mode);

                if (mode == 7 && prev != 7) {
                    command_gate_open_.store(false);
                    joints_stable_.store(false);
                    recovery_done_received_.store(false);
                    stable_count_ = 0;
                    RCLCPP_WARN(this->get_logger(), "Emergency stop detected, blocking joint_commands forwarding");
                } else if (mode != 7 && prev == 7) {
                    stable_count_ = 0;
                    RCLCPP_INFO(this->get_logger(), "Emergency stop cleared, waiting for stable joint_states + recovery notification...");
                }
            });

        // 订阅急停恢复完成通知
        auto recovery_qos = rclcpp::QoS(1).transient_local();
        estop_recovery_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/algorithm/internal/estop_recovery_done", recovery_qos,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    recovery_done_received_.store(true);
                    recovery_notification_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Received estop recovery done notification");

                    // 创建100ms后的一次性定时器来解除阻塞
                    unblock_delay_timer_ = this->create_wall_timer(
                        std::chrono::milliseconds(100),
                        [this]() {
                            tryUnblockCommands(true);  // skip_delay_check=true
                            unblock_delay_timer_.reset();  // 执行后清除定时器
                        });
                }
            });

        // 仿真模式末端位姿发布（仅 use_simulator_=true 时发布，只读 TF，不碰 joint_states/joint_commands）
        left_sim_tcp_pub_ = this->create_publisher<planning_sdk_msgs::msg::TcpPose>(
            "/algorithm/grasp_planning/left_arm/simulator/tcp_pose", 10);
        right_sim_tcp_pub_ = this->create_publisher<planning_sdk_msgs::msg::TcpPose>(
            "/algorithm/grasp_planning/right_arm/simulator/tcp_pose", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sim_tcp_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // 10Hz
            std::bind(&SimInterfaceSwitchNode::publishSimTcpPose, this));

        // 注册队列清空回调（在急停恢复时由 SharedResources 同步调用）
        if (shared_resources_) {
            shared_resources_->registerClearQueueCallback([this]() {
                RCLCPP_WARN(this->get_logger(), "Clearing joint_commands queue before recovery");
                joint_command_sub_.reset();
                joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
                    "/joint_commands", 10,
                    std::bind(&SimInterfaceSwitchNode::jointCommandCallback, this, std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(), "joint_commands queue cleared");
            });
        }

        RCLCPP_INFO(this->get_logger(), "Interface switch node initialized");
    }

private:
    void handleSwitchRequest(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        bool new_sim = request->data;
        if (new_sim == use_simulator_) {
            response->success = true;
            response->message = new_sim ? "Already in SIMULATOR mode" : "Already in REAL ROBOT mode";
            return;
        }

        if (new_sim) {
            // 实机 → 仿真：先停控制器，再切模式，防止控制器残余指令被 echo 到 /joint_states 导致飞车
            RCLCPP_INFO(this->get_logger(), "Switching to SIMULATOR mode, safe handover in progress...");
            std::thread([this]() {
                // Step1: 停控制器，防止残余指令污染 /joint_states
                deactivateControllers();
                // Step2: 切换模式，simJointStateCallback 开始向 /joint_states 注入仿真状态
                use_simulator_ = true;
                // Step3: 等待 TopicBasedSystem 从 /joint_states 刷新硬件接口（约100ms）
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                // Step4: 重启控制器，从当前位置初始化，误差=0
                activateControllers();
                RCLCPP_INFO(this->get_logger(), "Safe handover to SIMULATOR complete");
            }).detach();
            response->message = "Switching to simulator interface (safe handover in progress)";
        } else {
            RCLCPP_INFO(this->get_logger(), "Switching to REAL ROBOT mode, safe handover in progress...");
            std::thread([this]() {
                // Step1: 停控制器，清空内部积分状态
                deactivateControllers();
                // Step2: 切换模式，realJointStateCallback 开始向 /joint_states 注入真机状态
                use_simulator_ = false;
                // Step3: 等待真机状态通过 TopicBasedSystem 刷新到硬件接口（约200ms）
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                // Step4: 重启控制器，从真机当前位置初始化，误差=0，不产生突发指令
                activateControllers();
                RCLCPP_INFO(this->get_logger(), "Safe handover to REAL ROBOT complete");
            }).detach();
            response->message = "Switching to real robot interface (safe handover in progress)";
        }

        response->success = true;
    }

    void simJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto msg_cur = *msg;
        msg_cur.header.stamp = this->now();
        sim_joint_state_pub_->publish(msg_cur);
        if (use_simulator_)
        {
            joint_state_pub_->publish(msg_cur);
        }
    }

    void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto msg_cur = *msg;
        msg_cur.header.stamp = this->now();
        if (use_simulator_)
        {
            joint_state_pub_->publish(msg_cur);
        }
        else
        {
            if (!command_gate_open_.load()) {
                RCLCPP_DEBUG(this->get_logger(), "joint_commands blocked during emergency stop");
                return;
            }
            real_joint_command_pub_->publish(msg_cur);
        }
    }

    // 真机关节状态回调：真机模式下转发到 /joint_states，并处理急停恢复防抖
    void realJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!use_simulator_)
        {
            auto msg_cur = *msg;
            msg_cur.header.stamp = this->now();
            joint_state_pub_->publish(msg_cur);

            // 急停恢复防抖：等 joint_states 连续稳定后标记
            bool gate_closed = !command_gate_open_.load();
            int run_mode = current_run_mode_.load();
            if (gate_closed && run_mode != 7) {  // 非急停模式下检查稳定性
                if (!last_joint_state_positions_.empty() &&
                    last_joint_state_positions_.size() == msg_cur.position.size()) {
                    double max_diff = 0.0;
                    for (size_t i = 0; i < msg_cur.position.size(); ++i) {
                        max_diff = std::max(max_diff, std::fabs(msg_cur.position[i] - last_joint_state_positions_[i]));
                    }
                    // RCLCPP_INFO(this->get_logger(), "Estop recovery: max_diff=%.6f, stable_count=%d/%d",
                    //     max_diff, stable_count_, STABLE_FRAMES_REQUIRED);
                    if (max_diff < STABLE_THRESHOLD) {
                        ++stable_count_;
                    } else {
                        stable_count_ = 0;
                    }
                    if (stable_count_ >= STABLE_FRAMES_REQUIRED && !joints_stable_.load()) {
                        joints_stable_.store(true);
                        RCLCPP_INFO(this->get_logger(),
                            "joint_states stable (%d frames, max_diff < %.4f)",
                            STABLE_FRAMES_REQUIRED, STABLE_THRESHOLD);
                        tryUnblockCommands();
                    }
                } else {
                    RCLCPP_INFO(this->get_logger(), "Estop recovery: initializing last_joint_state_positions (size=%zu)", msg_cur.position.size());
                }
                last_joint_state_positions_ = msg_cur.position;
            } else if (gate_closed && run_mode == 7) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "Waiting for emergency stop to clear (run_mode=7)");
            }
        }
    }

    // 两个条件都满足时才放行
    void tryUnblockCommands(bool skip_delay_check = false)
    {
        // 添加调试日志：打印当前状态
        RCLCPP_INFO(this->get_logger(),
            "tryUnblockCommands: joints_stable=%d, recovery_done=%d, gate_open=%d, skip_delay=%d",
            joints_stable_.load(), recovery_done_received_.load(),
            command_gate_open_.load(), skip_delay_check);

        if (joints_stable_.load() && recovery_done_received_.load() && !command_gate_open_.load()) {
            // 检查是否已经过了100ms（确保控制器执行完锁定轨迹）
            // 定时器触发时跳过此检查（因为定时器本身已保证延迟）
            if (!skip_delay_check) {
                auto elapsed = (this->now() - recovery_notification_time_).seconds();
                if (elapsed < 0.1) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "Waiting for trajectory execution (%.3fs elapsed, need 0.1s)", elapsed);
                    return;
                }
            }

            command_gate_open_.store(true);
            RCLCPP_INFO(this->get_logger(),
                "Both conditions met + 100ms delay passed, unblocking joint_commands");
        } else {
            // 添加调试日志：说明为什么没有解锁
            if (command_gate_open_.load()) {
                RCLCPP_DEBUG(this->get_logger(), "Gate already open, skip unblock");
            }
        }
    }

    // 仿真模式末端位姿：只读 TF，use_simulator_=false 时直接返回，不产生任何输出
    void publishSimTcpPose()
    {
        if (!use_simulator_) return;

        auto lookup_and_publish = [this](
            const std::string & tip_frame,
            rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr & pub)
        {
            planning_sdk_msgs::msg::TcpPose msg;
            msg.timestamp = this->now();
            msg.error_code = 0;
            try {
                auto tf = tf_buffer_->lookupTransform(
                    "base_link", tip_frame, tf2::TimePointZero,
                    std::chrono::milliseconds(50));
                msg.current_pose.position.x = tf.transform.translation.x;
                msg.current_pose.position.y = tf.transform.translation.y;
                msg.current_pose.position.z = tf.transform.translation.z;
                msg.current_pose.orientation = tf.transform.rotation;
            } catch (const tf2::TransformException & ex) {
                RCLCPP_DEBUG(this->get_logger(), "TF lookup failed for %s: %s",
                             tip_frame.c_str(), ex.what());
                msg.error_code = 1;
            }
            pub->publish(msg);
        };

        lookup_and_publish("left_gripper_base_link",  left_sim_tcp_pub_);
        lookup_and_publish("right_gripper_base_link", right_sim_tcp_pub_);
    }

    // 停止轨迹控制器（清空内部积分状态）
    void deactivateControllers()
    {
        if (!switch_controller_cli_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_WARN(this->get_logger(), "controller_manager/switch_controller not available, skip deactivate");
            return;
        }
        auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        req->deactivate_controllers = TRAJECTORY_CONTROLLERS;
        req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        auto future = switch_controller_cli_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Controllers deactivated");
        } else {
            RCLCPP_WARN(this->get_logger(), "Deactivate controllers timeout");
        }
    }

    // 重新激活轨迹控制器（从硬件接口当前位置初始化，误差=0）
    void activateControllers()
    {
        if (!switch_controller_cli_->wait_for_service(std::chrono::seconds(3))) {
            RCLCPP_WARN(this->get_logger(), "controller_manager/switch_controller not available, skip activate");
            return;
        }
        auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        req->activate_controllers = TRAJECTORY_CONTROLLERS;
        req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        auto future = switch_controller_cli_->async_send_request(req);
        if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
            RCLCPP_INFO(this->get_logger(), "Controllers activated");
        } else {
            RCLCPP_WARN(this->get_logger(), "Activate controllers timeout");
        }
    }

    void restartControllers()
    {
        deactivateControllers();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        activateControllers();
    }

    static const inline std::vector<std::string> TRAJECTORY_CONTROLLERS = {
        "left_arm_group_controller",
        "right_arm_group_controller",
    };

    // 急停防抖参数
    static constexpr double STABLE_THRESHOLD = 0.01;   // 0.01弧度 ≈ 0.57度
    static constexpr int STABLE_FRAMES_REQUIRED = 3;   // 连续3帧

    bool use_simulator_;
    basic_control_topic::SharedResources::Ptr shared_resources_;
    std::atomic<bool> command_gate_open_{true};
    std::atomic<bool> joints_stable_{false};
    std::atomic<bool> recovery_done_received_{false};
    rclcpp::Time recovery_notification_time_;  // 收到恢复通知的时间
    rclcpp::TimerBase::SharedPtr unblock_delay_timer_;  // 延迟解除阻塞的定时器
    std::atomic<int> current_run_mode_{0};
    int stable_count_ = 0;
    std::vector<double> last_joint_state_positions_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_service_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_cli_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr real_joint_command_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_joint_state_pub_;
    rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr left_sim_tcp_pub_;
    rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr right_sim_tcp_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_joint_state_sub_;
    rclcpp::Subscription<cerebellum_sdk_msg::msg::MotorState>::SharedPtr motor_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_recovery_done_sub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr sim_tcp_timer_;
};

// ============================================================================
// 辅助节点：帧注册服务器
// ============================================================================
class FrameRegistrationServerNode : public rclcpp::Node
{
public:
    FrameRegistrationServerNode() : Node("frame_registration_server")
    {
        service_ = this->create_service<planning_sdk_msgs::srv::RegisterFrame>(
            "/algorithm/grasp_planning/register_frame",
            std::bind(&FrameRegistrationServerNode::handleRequest, this, std::placeholders::_1, std::placeholders::_2));

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(), "Frame registration server ready");
    }

private:
    void handleRequest(
        const std::shared_ptr<planning_sdk_msgs::srv::RegisterFrame::Request> request,
        std::shared_ptr<planning_sdk_msgs::srv::RegisterFrame::Response> response)
    {
        response->timestamp = this->now();

        // 验证请求
        if (request->frame.header.frame_id.empty()) {
            response->error_code = 400;
            RCLCPP_ERROR(this->get_logger(), "Parent frame ID is empty");
            return;
        }

        if (request->frame.child_frame_id.empty()) {
            response->error_code = 401;
            RCLCPP_ERROR(this->get_logger(), "Child frame ID is empty");
            return;
        }

        // 发布静态TF变换
        std::vector<geometry_msgs::msg::TransformStamped> transforms = {request->frame};
        tf_broadcaster_->sendTransform(transforms);

        response->error_code = 0;
        RCLCPP_INFO(this->get_logger(), "Registered frame: %s -> %s",
                    request->frame.header.frame_id.c_str(),
                    request->frame.child_frame_id.c_str());
    }

    rclcpp::Service<planning_sdk_msgs::srv::RegisterFrame>::SharedPtr service_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

// ============================================================================
// Main
// ============================================================================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 创建共享资源
    auto shared_resources = std::make_shared<basic_control_topic::SharedResources>();

    // 创建多线程执行器
    rclcpp::executors::MultiThreadedExecutor executor;

    // 创建所有节点
    // 1. 关节限位服务器（首先创建，初始化共享资源）
    auto joint_limits_node = std::make_shared<JointLimitsServerNode>(shared_resources);
    shared_resources->initialize(joint_limits_node);

    // 2. 位置控制服务器
    auto position_servers = std::make_shared<basic_control_topic::PositionServersNode>(shared_resources);

    // 3. 速度控制服务器
    auto velocity_controllers = std::make_shared<basic_control_topic::VelocityControllersNode>(shared_resources);

    // 4. TCP位姿发布
    auto tcp_pose_publisher = std::make_shared<TcpPosePublisherNode>();

    // 5. 心跳
    auto heart_beat = std::make_shared<HeartBeatNode>();

    // 6. Servo管理
    auto servo_manager = std::make_shared<ServoManagerNode>(shared_resources);

    // 7. 仿真接口切换
    auto sim_interface_switch = std::make_shared<SimInterfaceSwitchNode>(shared_resources);

    // 8. 帧注册服务器
    auto frame_registration_server = std::make_shared<FrameRegistrationServerNode>();

    // 添加所有节点到执行器
    executor.add_node(joint_limits_node);
    executor.add_node(position_servers);
    executor.add_node(velocity_controllers);
    executor.add_node(tcp_pose_publisher);
    executor.add_node(heart_beat);
    executor.add_node(servo_manager);
    executor.add_node(sim_interface_switch);
    executor.add_node(frame_registration_server);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Basic Control Node started with 8 sub-nodes in single process");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - joint_limits_server");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - position_servers (joint_position, tcp_position, joint_delta, tcp_delta)");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - velocity_controllers (tcp_velocity, joint_velocity)");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - tcp_pose_publisher");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - heart_beat");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - servo_manager");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - sim_interface_switch");
    RCLCPP_INFO(rclcpp::get_logger("main"), "  - frame_registration_server");

    // 执行
    executor.spin();

    rclcpp::shutdown();
    return 0;
}