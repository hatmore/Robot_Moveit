#include <rclcpp/rclcpp.hpp>
#include <planning_sdk_msgs/msg/tcp_velocity_once.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <map>



class TcpServo
{
public:
    TcpServo(const std::string& arm)
    : arm_(arm)
    {
        arm_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
            "/algorithm/grasp_planning/move/" + arm_ + "_arm/tcp_velocity_once",
            10,
            std::bind(&TcpVelocityToServo::armVelocityCallback, this, std::placeholders::_1));
        
        arm_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/" + arm_ + "_arm_servo_node/delta_twist_cmds",
            10);
        
        RCLCPP_INFO(this->get_logger(), arm_ + " TCP速度控制已启用");
    }

private:
    std::string arm_;
}




class TcpVelocityToServo : public rclcpp::Node
{
public:
    explicit TcpVelocityToServo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("tcp_velocity_to_servo", options),
          right_arm_publisher_connected_(false),
          left_arm_publisher_connected_(false)
    {
        // 声明参数
        this->declare_parameter<bool>("enable_left_arm", true);
        this->declare_parameter<bool>("enable_right_arm", true);
        this->declare_parameter<double>("max_linear_velocity", 0.3);
        this->declare_parameter<double>("max_angular_velocity", 0.5);
        this->declare_parameter<bool>("verbose_logging", true);
        
        // 获取参数
        bool enable_left_arm = this->get_parameter("enable_left_arm").as_bool();
        bool enable_right_arm = this->get_parameter("enable_right_arm").as_bool();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        verbose_logging_ = this->get_parameter("verbose_logging").as_bool();
        
        // 初始化命令历史记录
        repeated_command_count_["left"] = 0;
        repeated_command_count_["right"] = 0;
        singularity_warning_count_["left"] = 0;
        singularity_warning_count_["right"] = 0;
        
        // 创建诊断发布者
        diagnostic_publisher_ = this->create_publisher<std_msgs::msg::String>("/tcp_velocity_diagnostics", 10);
        
        // 创建左臂的订阅者和发布者
        if (enable_left_arm) {
            
        }
        
        // 创建右臂的订阅者和发布者
        if (enable_right_arm) {
            
        }
        
        connection_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&TcpVelocityToServo::checkConnections, this));
        
        RCLCPP_INFO(this->get_logger(), "TCP Velocity to Servo Node Started");
        RCLCPP_INFO(this->get_logger(), "Max Linear Velocity: %.3f m/s", max_linear_velocity_);
        RCLCPP_INFO(this->get_logger(), "Max Angular Velocity: %.3f rad/s", max_angular_velocity_);
        RCLCPP_INFO(this->get_logger(), "Verbose Logging: %s", verbose_logging_ ? "ON" : "OFF");
        
        if (!enable_left_arm && !enable_right_arm) {
            RCLCPP_WARN(this->get_logger(), "左右臂均未启用，请检查参数配置");
        }
    }

private:
    void leftArmVelocityCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg)
    {
        if (!left_arm_publisher_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "左臂Servo节点未连接，忽略速度命令");
            return;
        }
        processVelocityCommand(msg, "left", left_arm_publisher_);
    }
    
    void rightArmVelocityCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg)
    {
        if (!right_arm_publisher_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "右臂Servo节点未连接，忽略速度命令");
            return;
        }
        processVelocityCommand(msg, "right", right_arm_publisher_);
    }
    
    void checkConnections()
    {
        auto check_publisher_connection = [this](const std::string& arm_side, 
                                                const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& publisher,
                                                bool& connected_flag) {
            if (publisher) {
                auto subscribers = publisher->get_subscription_count();
                if (subscribers > 0 && !connected_flag) {
                    connected_flag = true;
                    RCLCPP_INFO(this->get_logger(), "%s臂Servo节点已连接 (订阅者数量: %d)", 
                               arm_side.c_str(), subscribers);
                    publishDiagnostic(arm_side + "臂Servo节点已连接");
                } else if (subscribers == 0 && connected_flag) {
                    connected_flag = false;
                    RCLCPP_WARN(this->get_logger(), "%s臂Servo节点连接丢失", arm_side.c_str());
                    publishDiagnostic(arm_side + "臂Servo节点连接丢失");
                }
            }
        };
        
        check_publisher_connection("左", left_arm_publisher_, left_arm_publisher_connected_);
        check_publisher_connection("右", right_arm_publisher_, right_arm_publisher_connected_);
    }
    
    void publishDiagnostic(const std::string& message)
    {
        auto diagnostic_msg = std_msgs::msg::String();
        diagnostic_msg.data = message;
        diagnostic_publisher_->publish(diagnostic_msg);
    }
    
    void processVelocityCommand(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg, 
                               const std::string& arm_side,
                               const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& publisher)
    {
        if (singularity_warning_count_[arm_side] > 5) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "%s机械臂持续处于奇异点附近，请手动调整机械臂姿态", arm_side.c_str());
            return;
        }
        
        if (repeated_command_count_[arm_side] % 10 == 0 && verbose_logging_) {
            RCLCPP_INFO(this->get_logger(), "收到%s臂TCP速度控制命令:", arm_side.c_str());
            RCLCPP_INFO(this->get_logger(), "  时间戳: %d.%09d", 
                       msg->timestamp.sec, msg->timestamp.nanosec);
            RCLCPP_INFO(this->get_logger(), "  旋转样式: %d (%s)", 
                       msg->rotation_style, getRotationStyleName(msg->rotation_style).c_str());
            RCLCPP_INFO(this->get_logger(), "  速度百分比: " );
            RCLCPP_INFO(this->get_logger(), "      x: %f", msg->velocity.linear.x);
            RCLCPP_INFO(this->get_logger(), "      y: %f", msg->velocity.linear.y);
            RCLCPP_INFO(this->get_logger(), "      z: %f", msg->velocity.linear.z);
            RCLCPP_INFO(this->get_logger(), "      rx: %f", msg->velocity.angular.x);
            RCLCPP_INFO(this->get_logger(), "      ry: %f", msg->velocity.angular.y);
            RCLCPP_INFO(this->get_logger(), "      rz: %f", msg->velocity.angular.z);
        } else if (verbose_logging_) {
            RCLCPP_DEBUG(this->get_logger(), "%s臂持续速度控制 (重复次数: %d)", 
                        arm_side.c_str(), repeated_command_count_[arm_side]);
        }
        
        try
        {
            auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            twist_msg->header.stamp = this->now();
            twist_msg->header.frame_id = "base_link";  
            

            twist_msg->twist = msg->velocity;
            twist_msg->twist.linear.x *= max_linear_velocity_;
            twist_msg->twist.linear.y *= max_linear_velocity_;
            twist_msg->twist.linear.z *=  max_linear_velocity_;
            
            twist_msg->twist.angular.x *= max_angular_velocity_;
            twist_msg->twist.angular.y *= max_angular_velocity_;
            twist_msg->twist.angular.z *= max_angular_velocity_;
            
            if (repeated_command_count_[arm_side] % 10 == 0 && verbose_logging_) {
                RCLCPP_INFO(this->get_logger(), "%s臂转换后的实际速度:", arm_side.c_str());
                RCLCPP_INFO(this->get_logger(), "  线速度 [m/s]:");
                RCLCPP_INFO(this->get_logger(), "    X: %8.4f", twist_msg->twist.linear.x);
                RCLCPP_INFO(this->get_logger(), "    Y: %8.4f", twist_msg->twist.linear.y);
                RCLCPP_INFO(this->get_logger(), "    Z: %8.4f", twist_msg->twist.linear.z);
                RCLCPP_INFO(this->get_logger(), "  角速度 [rad/s]:");
                RCLCPP_INFO(this->get_logger(), "    X: %8.4f", twist_msg->twist.angular.x);
                RCLCPP_INFO(this->get_logger(), "    Y: %8.4f", twist_msg->twist.angular.y);
                RCLCPP_INFO(this->get_logger(), "    Z: %8.4f", twist_msg->twist.angular.z);
            }
            
            // 发布到对应的Servo节点
            publisher->publish(std::move(twist_msg));
            
            if (repeated_command_count_[arm_side] % 10 == 0 && verbose_logging_) {
                RCLCPP_INFO(this->get_logger(), "成功发布%s臂Servo速度命令", arm_side.c_str());
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "处理%s臂速度命令时发生错误: %s", arm_side.c_str(), e.what());
        }
    }
    
    std::string getRotationStyleName(uint16_t rotation_style)
    {
        switch (rotation_style)
        {
            case 0: return "外接旋转(基坐标系)";
            case 1: return "内旋(XYZ)";
            case 2: return "内旋(ZYX)";
            default: return "未知旋转样式";
        }
    }
    
    std::vector<double> parseDirectString(const std::string& direct_str)
    {
        std::vector<double> components;
        std::stringstream ss(direct_str);
        std::string item;
        
        if (verbose_logging_) {
            RCLCPP_DEBUG(this->get_logger(), "解析方向字符串: '%s'", direct_str.c_str());
        }
        
        while (std::getline(ss, item, ','))
        {
            try
            {
                item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
                
                if (!item.empty() && std::isalpha(item[0]))
                {
                    double value = 0.0;
                    if (item == "x" || item == "px") value = 1.0;
                    else if (item == "-x" || item == "-px") value = -1.0;
                    else if (item == "y" || item == "ry") value = 1.0;
                    else if (item == "-y" || item == "-ry") value = -1.0;
                    else if (item == "z" || item == "rz") value = 1.0;
                    else if (item == "-z" || item == "-rz") value = -1.0;
                    else value = 0.0;
                    components.push_back(value);
                }
                else
                {
                    double value = std::stod(item);
                    components.push_back(value);
                }
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "解析组件失败: '%s', 错误: %s", item.c_str(), e.what());
                components.push_back(0.0);
            }
        }
        
        if (components.size() != 6)
        {
            RCLCPP_WARN(this->get_logger(), "方向字符串分量数量不正确: 期望6个, 实际%zu个", components.size());
            components.resize(6, 0.0);
        }
        
        return components;
    }
    
    std::vector<double> applyRotationStyle(
        const std::vector<double>& angular_components, 
        uint16_t rotation_style)
    {
        std::vector<double> result = angular_components;
        
        if (verbose_logging_) {
            RCLCPP_DEBUG(this->get_logger(), "应用旋转样式: %s", getRotationStyleName(rotation_style).c_str());
        }
        
        switch (rotation_style)
        {
            case 0: break;
            case 1: 
            case 2: 
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "未知旋转样式: %d", rotation_style);
                break;
        }
        
        return result;
    }

    double max_linear_velocity_;
    double max_angular_velocity_;
    bool verbose_logging_;

    bool right_arm_publisher_connected_;
    bool left_arm_publisher_connected_;
    
    std::map<std::string, int> repeated_command_count_;
    std::map<std::string, int> singularity_warning_count_;

    rclcpp::TimerBase::SharedPtr connection_check_timer_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_publisher_;

    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr left_arm_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr left_arm_publisher_;
     
    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr right_arm_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr right_arm_publisher_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("main"), "启动TCP速度到Servo转换器");
    auto node = std::make_shared<TcpVelocityToServo>();
    RCLCPP_INFO(node->get_logger(), "节点已启动，等待速度控制命令");
    rclcpp::spin(node);
    RCLCPP_INFO(rclcpp::get_logger("main"), "关闭TCP速度到Servo转换器");
    rclcpp::shutdown();
    return 0;
}