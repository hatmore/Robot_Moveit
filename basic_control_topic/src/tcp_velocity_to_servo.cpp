/**
 * ================================================================================
 * TCP 速度到 Servo 转换器 (TCP Velocity to Servo)
 * ================================================================================
 *
 * 功能: 将百分比形式的 TCP 速度命令转换为 TwistStamped 消息，发送给伺服控制器
 *
 * 核心特点:
 *   - 百分比速度转换：输入 0-1 范围的百分比，乘以最大速度限制得到实际速度
 *   - 动态连接检测：每 3 秒检查一次 Servo 节点是否已连接
 *   - 命令节流：每 10 个命令打印详细日志，防止日志淹没终端
 *   - 双臂独立控制：左臂和右臂的速度命令独立发送
 *
 * 通信方式（与 Action 不同）:
 *   - Action：同步通信，客户端等待服务器完成
 *   - Servo：异步流式通信，客户端不断发送速度命令
 *   - 特点：低延迟、可中断（停止发送 = 停止运动）、无返回值反馈
 *
 * 应用场景:
 *   - 点动控制：用户通过按钮发送速度脉冲
 *   - 实时速度调节：频繁改变运动速度
 *   - 交互式控制：快速响应用户输入
 *
 * 参数配置:
 *   enable_left_arm: 是否启用左臂控制（默认 true）
 *   enable_right_arm: 是否启用右臂控制（默认 true）
 *   max_linear_velocity: 最大线速度 [m/s]（默认 0.3）
 *   max_angular_velocity: 最大角速度 [rad/s]（默认 0.5）
 *   verbose_logging: 是否输出详细日志（默认 true）
 *
 * ================================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <planning_sdk_msgs/msg/tcp_velocity_once.hpp>
#include <planning_sdk_msgs/msg/joint_limits_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <cctype>
#include <iomanip>
#include <map>
#include <mutex>

class TcpVelocityToServo : public rclcpp::Node
{
public:
    /**
     * 构造函数: 初始化 TCP 速度到 Servo 转换器
     *
     * 初始化过程:
     *   1. 声明并读取 5 个参数（启用标志、速度限制、日志开关）
     *   2. 初始化命令计数器和警告计数器（用于节流日志）
     *   3. 如果启用左臂：创建左臂订阅者和发布者
     *   4. 如果启用右臂：创建右臂订阅者和发布者
     *   5. 创建诊断发布者（用于连接状态通知）
     *   6. 创建周期性 Timer（每 3 秒执行一次连接检查）
     *
     * 调用时机: 程序启动时
     * 输入: 节点选项（可指定节点名、命名空间等）
     * 输出: Node 就绪，开始监听速度命令
     *
     * 关键成员初始化:
     *   - repeated_command_count_[\"left\"] 和 [\"right\"]：命令计数
     *   - singularity_warning_count_[\"left\"] 和 [\"right\"]：奇异点警告计数
     *   - left_arm_publisher_connected_ 和 right_arm_publisher_connected_：连接标志
     */
    explicit TcpVelocityToServo(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("tcp_velocity_to_servo", options),
          right_arm_publisher_connected_(false),
          left_arm_publisher_connected_(false)
    {
        // 声明参数
        this->declare_parameter<bool>("enable_left_arm", true);
        this->declare_parameter<bool>("enable_right_arm", true);
        this->declare_parameter<double>("max_linear_velocity", 2.0);
        this->declare_parameter<double>("max_angular_velocity", 2.5);
        this->declare_parameter<bool>("verbose_logging", true);
        this->declare_parameter<double>("decel_duration", 0.3);       // 减速持续时间(秒)
        this->declare_parameter<int>("decel_steps", 10);               // 减速分几步(100Hz下=100ms)
        this->declare_parameter<double>("command_timeout", 0.15);     // 指令超时判定(秒)

        // 获取参数
        bool enable_left_arm = this->get_parameter("enable_left_arm").as_bool();
        bool enable_right_arm = this->get_parameter("enable_right_arm").as_bool();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        verbose_logging_ = this->get_parameter("verbose_logging").as_bool();
        decel_duration_ = this->get_parameter("decel_duration").as_double();
        decel_steps_ = this->get_parameter("decel_steps").as_int();
        command_timeout_ = this->get_parameter("command_timeout").as_double();
        
        // 初始化关节列表及默认位置限位
        std::vector<std::string> all_joints = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
            "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
            "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint"
        };
        for (const auto& jname : all_joints) {
            joint_min_positions_[jname] = -3.14159;
            joint_max_positions_[jname] = 3.14159;
            current_joint_positions_[jname] = 0.0;
        }

        // 订阅动态关节限位（transient_local，与 server 端 QoS 匹配）
        auto limits_qos = rclcpp::QoS(1).transient_local();
        joint_limits_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointLimitsArray>(
            "/algorithm/joint_limits/current", limits_qos,
            std::bind(&TcpVelocityToServo::jointLimitsCallback, this, std::placeholders::_1));

        // 订阅关节状态（用于位置限位）
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TcpVelocityToServo::jointStateCallback, this, std::placeholders::_1));

        // 初始化命令历史记录
        repeated_command_count_["left"] = 0;
        repeated_command_count_["right"] = 0;
        singularity_warning_count_["left"] = 0;
        singularity_warning_count_["right"] = 0;
        
        // 创建诊断发布者
        diagnostic_publisher_ = this->create_publisher<std_msgs::msg::String>("/tcp_velocity_diagnostics", 10);
        
        // 创建左臂的订阅者和发布者
        if (enable_left_arm) {
            left_arm_subscription_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
                "/algorithm/grasp_planning/move/left_arm/tcp_velocity_once",
                10,
                std::bind(&TcpVelocityToServo::leftArmVelocityCallback, this, std::placeholders::_1));
            
            // QoS 与 servo 节点的 SensorDataQoS 订阅匹配
            auto left_qos = rclcpp::SensorDataQoS();
            left_arm_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/left_arm_servo_node/delta_twist_cmds",
                left_qos);
            
            left_arm_status_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
                "/left_arm_servo_node/status", 10,
                [this](const std_msgs::msg::Int8::SharedPtr msg) { servoStatusCallback(msg, "左"); });

            RCLCPP_INFO(this->get_logger(), "左臂TCP速度控制已启用");
        }

        // 创建右臂的订阅者和发布者
        if (enable_right_arm) {
            right_arm_subscription_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
                "/algorithm/grasp_planning/move/right_arm/tcp_velocity_once",
                10,
                std::bind(&TcpVelocityToServo::rightArmVelocityCallback, this, std::placeholders::_1));
            
            auto right_qos = rclcpp::SensorDataQoS();
            right_arm_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
                "/right_arm_servo_node/delta_twist_cmds",
                right_qos);
            
            right_arm_status_subscription_ = this->create_subscription<std_msgs::msg::Int8>(
                "/right_arm_servo_node/status", 10,
                [this](const std_msgs::msg::Int8::SharedPtr msg) { servoStatusCallback(msg, "右"); });

            RCLCPP_INFO(this->get_logger(), "右臂TCP速度控制已启用");
        }
        
        connection_check_timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            std::bind(&TcpVelocityToServo::checkConnections, this));

        // 减速检测定时器，10ms 周期与 servo 100Hz 对齐
        decel_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TcpVelocityToServo::decelCheckCallback, this));
        
        RCLCPP_INFO(this->get_logger(), "TCP Velocity to Servo Node Started");
        RCLCPP_INFO(this->get_logger(), "Max Linear Velocity: %.3f m/s", max_linear_velocity_);
        RCLCPP_INFO(this->get_logger(), "Max Angular Velocity: %.3f rad/s", max_angular_velocity_);
        RCLCPP_INFO(this->get_logger(), "Verbose Logging: %s", verbose_logging_ ? "ON" : "OFF");
        
        if (!enable_left_arm && !enable_right_arm) {
            RCLCPP_WARN(this->get_logger(), "左右臂均未启用，请检查参数配置");
        }
    }

private:
    /**
     * 左臂 TCP 速度命令回调
     *
     * 调用时机: 每当收到 /algorithm/grasp_planning/move/left_arm/tcp_velocity_once 消息时触发
     * 输入: msg - TcpVelocityOnce 消息（速度百分比、旋转样式、时间戳）
     * 输出: 检查连接后调用 processVelocityCommand() 处理
     *
     * 防护措施:
     *   - 检查 left_arm_publisher_connected_ 标志，未连接时忽略命令
     *   - 检查 left_arm_publisher_ 发布者是否已初始化
     *   - 输出 WARN_THROTTLE 日志，5 秒内最多输出一次（防止日志淹没）
     */
    void leftArmVelocityCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg)
    {
        if (!left_arm_publisher_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "左臂Servo节点未连接，忽略速度命令");
            return;
        }
        if (!left_arm_publisher_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "左臂发布者未初始化");
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
        if (!right_arm_publisher_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "右臂发布者未初始化");
            return;
        }
        processVelocityCommand(msg, "right", right_arm_publisher_);
    }

    void jointLimitsCallback(const planning_sdk_msgs::msg::JointLimitsArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (const auto& jl : msg->limits) {
            if (joint_min_positions_.count(jl.joint_name)) {
                joint_min_positions_[jl.joint_name] = jl.lower_limit;
                joint_max_positions_[jl.joint_name] = jl.upper_limit;
            }
        }
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = current_joint_positions_.find(msg->name[i]);
            if (it != current_joint_positions_.end()) {
                it->second = msg->position[i];
            }
        }
    }

    // TCP 速度限位缩放：接近限位时线性缩减，已超限时全部屏蔽
    // 注意：TCP 速度无法判断哪个方向能让关节回到限位内（需要 Jacobian），
    // 因此超限后屏蔽全部 TCP 运动，需通过关节速度控制恢复。
    double computePositionLimitScale() {
        constexpr double kBuffer = 0.05;
        double scale = 1.0;
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (const auto& [name, pos] : current_joint_positions_) {
            double min_pos = joint_min_positions_[name];
            double max_pos = joint_max_positions_[name];
            double dist_upper = max_pos - pos;
            double dist_lower = pos - min_pos;

            // 已超出限位：屏蔽全部 TCP 速度
            if (dist_upper < 0.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "关节[%s] 已超出上限位 (pos=%.4f > max=%.4f)，屏蔽全部TCP速度",
                    name.c_str(), pos, max_pos);
                return 0.0;
            }
            if (dist_lower < 0.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "关节[%s] 已超出下限位 (pos=%.4f < min=%.4f)，屏蔽全部TCP速度",
                    name.c_str(), pos, min_pos);
                return 0.0;
            }

            // 接近限位（限位内缓冲区）：线性缩减速度
            if (dist_upper < kBuffer) {
                scale = std::min(scale, dist_upper / kBuffer);
            }
            if (dist_lower < kBuffer) {
                scale = std::min(scale, dist_lower / kBuffer);
            }
        }
        return std::max(scale, 0.0);
    }

    /**
     * Timer 回调：定期检查 Servo 节点连接状态
     *
     * 调用时机: 每 3 秒执行一次（由 Timer 触发）
     * 输入: 无
     * 输出: 更新 left_arm_publisher_connected_ 和 right_arm_publisher_connected_ 标志
     *
     * 检查原理:
     *   - 获取发布者的订阅者数量：publisher->get_subscription_count()
     *   - 如果订阅者数 > 0 且尚未标记为已连接：设置标志为 true，输出 INFO
     *   - 如果订阅者数 == 0 且已标记为已连接：设置标志为 false，输出 WARN
     *   - 发布诊断消息到 /tcp_velocity_diagnostics 话题
     *
     * 为什么需要这个检查？
     *   - Servo 节点可能崩溃或重启
     *   - 发布者维持连接但无订阅者时，消息会被丢弃
     *   - 及时检测连接状态，帮助用户调试
     */
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

    void decelCheckCallback()
    {
        auto check_and_decel = [this](const std::string& arm_side,
                                       const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& publisher,
                                       bool connected) {
            if (!publisher || !connected) return;
            if (last_cmd_time_.find(arm_side) == last_cmd_time_.end()) return;

            double elapsed = (this->now() - last_cmd_time_[arm_side]).seconds();

            // 指令超时且还没开始减速
            if (elapsed > command_timeout_ && !is_decelerating_[arm_side] && decel_step_count_[arm_side] == 0) {
                is_decelerating_[arm_side] = true;
                decel_step_count_[arm_side] = 0;
                RCLCPP_DEBUG(this->get_logger(), "%s臂指令超时，开始平滑停车", arm_side.c_str());
            }

            // 正在减速中，发送递减速度
            if (is_decelerating_[arm_side] && decel_step_count_[arm_side] < decel_steps_) {
                decel_step_count_[arm_side]++;
                double ratio = 1.0 - static_cast<double>(decel_step_count_[arm_side]) / decel_steps_;

                auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
                twist_msg->header.stamp = this->now();
                twist_msg->header.frame_id = "base_link";

                twist_msg->twist.linear.x = last_twist_[arm_side].linear.x * (1.0/100.0) * max_linear_velocity_ * ratio;
                twist_msg->twist.linear.y = last_twist_[arm_side].linear.y * (1.0/100.0) * max_linear_velocity_ * ratio;
                twist_msg->twist.linear.z = last_twist_[arm_side].linear.z * (1.0/100.0) * max_linear_velocity_ * ratio;
                twist_msg->twist.angular.x = last_twist_[arm_side].angular.x * (1.0/100.0) * max_angular_velocity_ * ratio;
                twist_msg->twist.angular.y = last_twist_[arm_side].angular.y * (1.0/100.0) * max_angular_velocity_ * ratio;
                twist_msg->twist.angular.z = last_twist_[arm_side].angular.z * (1.0/100.0) * max_angular_velocity_ * ratio;

                publisher->publish(std::move(twist_msg));
            } else if (is_decelerating_[arm_side] && decel_step_count_[arm_side] >= decel_steps_) {
                is_decelerating_[arm_side] = false;
            }
        };

        check_and_decel("left", left_arm_publisher_, left_arm_publisher_connected_);
        check_and_decel("right", right_arm_publisher_, right_arm_publisher_connected_);
    }

    void servoStatusCallback(const std_msgs::msg::Int8::SharedPtr msg, const std::string& arm_side)
    {
        int8_t status = msg->data;
        // 只在状态变化时或非正常状态时打印日志
        auto& last = last_servo_status_[arm_side];
        if (status == last && status == 0) {
            return;  // 正常状态不重复打印
        }
        last = status;

        switch (status) {
            case 0:
                RCLCPP_INFO(this->get_logger(), "[%s臂Servo] 状态恢复正常", arm_side.c_str());
                break;
            case 1:
                RCLCPP_WARN(this->get_logger(), "[%s臂Servo] 接近奇异点，正在减速", arm_side.c_str());
                break;
            case 2:
                RCLCPP_ERROR(this->get_logger(), "[%s臂Servo] 非常接近奇异点，硬停止!", arm_side.c_str());
                break;
            case 3:
                RCLCPP_WARN(this->get_logger(), "[%s臂Servo] 接近碰撞，正在减速", arm_side.c_str());
                break;
            case 4:
                RCLCPP_ERROR(this->get_logger(), "[%s臂Servo] 检测到碰撞，硬停止!", arm_side.c_str());
                break;
            case 5:
                RCLCPP_WARN(this->get_logger(), "[%s臂Servo] 接近关节限位，停止", arm_side.c_str());
                break;
            case 6:
                RCLCPP_WARN(this->get_logger(), "[%s臂Servo] 正在离开奇异点，减速中", arm_side.c_str());
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "[%s臂Servo] 未知状态码: %d", arm_side.c_str(), status);
                break;
        }
    }

    /**
     * 处理 TCP 速度命令：核心的速度转换与发布逻辑
     *
     * 调用时机: 左臂或右臂的速度回调触发时调用
     * 输入:
     *   - msg: TcpVelocityOnce 消息（百分比速度、旋转样式、时间戳）
     *   - arm_side: "left" 或 "right"（用于日志和记录）
     *   - publisher: 目标发布者（将转换后的速度发送给 Servo 节点）
     * 输出: 发布 TwistStamped 消息到 Servo 节点
     *
     * 处理过程（5 步）:
     *
     *   【步骤 1】命令计数与奇异点检查
     *     - 增加 repeated_command_count_[arm_side] 计数
     *     - 检查 singularity_warning_count_[arm_side] 是否 > 5
     *     - 如果超过，输出错误日志并返回（保护机械臂）
     *
     *   【步骤 2】节流日志输出
     *     - 仅当 repeated_command_count_ % 10 == 0 时才输出详细日志
     *     - 这样每收到 10 个命令才打印一次（防止日志淹没）
     *     - 日志包含：时间戳、旋转样式、百分比速度值
     *
     *   【步骤 3】百分比到实际速度的转换
     *     - 线速度：twist.linear.xyz = msg->velocity.linear.xyz * max_linear_velocity_
     *     - 角速度：twist.angular.xyz = msg->velocity.angular.xyz * max_angular_velocity_
     *
     *     示例:
     *       输入：msg->velocity.linear.x = 0.5（50%）
     *       max_linear_velocity_ = 0.3 m/s
     *       输出：twist.linear.x = 0.5 * 0.3 = 0.15 m/s
     *
     *   【步骤 4】设置消息头信息
     *     - header.stamp = now()（当前时刻）
     *     - header.frame_id = "base_link"（参考坐标系）
     *
     *   【步骤 5】发布到 Servo 节点
     *     - publisher->publish(twist_msg)（异步发布）
     *     - 使用 std::move(twist_msg) 避免不必要的拷贝
     *
     * 错误处理:
     *   - try-catch 捕获异常（如发布失败）
     *   - 异常时输出 ERROR 日志但不中止程序
     */
    void processVelocityCommand(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg,
                               const std::string& arm_side,
                               const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& publisher)
    {
        repeated_command_count_[arm_side]++;

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
            // 检测是否从静止状态启动
            bool was_idle = (last_cmd_time_.find(arm_side) == last_cmd_time_.end()) ||
                            is_decelerating_[arm_side] ||
                            (this->now() - last_cmd_time_[arm_side]).seconds() > command_timeout_;

            if (was_idle) {
                accel_start_time_[arm_side] = this->now();
                is_decelerating_[arm_side] = false;
                decel_step_count_[arm_side] = 0;
                RCLCPP_DEBUG(this->get_logger(), "%s臂从静止启动，开始平滑加速", arm_side.c_str());
            }

            // 加速斜坡：基于时间，而非命令计数（避免低频发命令时斜坡过长）
            double accel_ratio = 1.0;
            constexpr double kAccelDuration = 0.15;  // 加速时间固定0.15s，与command_timeout一致
            if (was_idle || accel_start_time_.find(arm_side) != accel_start_time_.end()) {
                double elapsed_since_start = (this->now() - accel_start_time_[arm_side]).seconds();
                if (elapsed_since_start < kAccelDuration) {
                    accel_ratio = elapsed_since_start / kAccelDuration;
                }
            }

            auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            twist_msg->header.stamp = this->now();
            twist_msg->header.frame_id = "base_link";

            // 输入范围 -100~100，归一化到 -1~1 再乘以最大速度
            constexpr double kVelocityScale = 1.0 / 100.0;
            twist_msg->twist = msg->velocity;
            twist_msg->twist.linear.x *= kVelocityScale * max_linear_velocity_ * accel_ratio;
            twist_msg->twist.linear.y *= kVelocityScale * max_linear_velocity_ * accel_ratio;
            twist_msg->twist.linear.z *= kVelocityScale * max_linear_velocity_ * accel_ratio;

            twist_msg->twist.angular.x *= kVelocityScale * max_angular_velocity_ * accel_ratio;
            twist_msg->twist.angular.y *= kVelocityScale * max_angular_velocity_ * accel_ratio;
            twist_msg->twist.angular.z *= kVelocityScale * max_angular_velocity_ * accel_ratio;

            // 应用位置限位缩放
            double pos_scale = computePositionLimitScale();
            if (pos_scale < 1.0) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                    "%s臂接近关节位置限位，速度缩放: %.2f", arm_side.c_str(), pos_scale);
            }
            twist_msg->twist.linear.x  *= pos_scale;
            twist_msg->twist.linear.y  *= pos_scale;
            twist_msg->twist.linear.z  *= pos_scale;
            twist_msg->twist.angular.x *= pos_scale;
            twist_msg->twist.angular.y *= pos_scale;
            twist_msg->twist.angular.z *= pos_scale;

            if (repeated_command_count_[arm_side] % 50 == 0 && verbose_logging_) {
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

            // 记录最后一次指令，用于平滑停车
            last_cmd_time_[arm_side] = this->now();
            last_twist_[arm_side] = msg->velocity;
            is_decelerating_[arm_side] = false;
            decel_step_count_[arm_side] = 0;
            
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

    // 关节位置限位
    std::map<std::string, double> joint_min_positions_;
    std::map<std::string, double> joint_max_positions_;
    std::map<std::string, double> current_joint_positions_;
    std::mutex joint_state_mutex_;

    std::map<std::string, int> repeated_command_count_;
    std::map<std::string, int> singularity_warning_count_;

    rclcpp::TimerBase::SharedPtr connection_check_timer_;
    rclcpp::TimerBase::SharedPtr decel_timer_;

    // 平滑停车相关
    double decel_duration_;
    int decel_steps_;
    double command_timeout_;
    std::map<std::string, rclcpp::Time> last_cmd_time_;
    std::map<std::string, geometry_msgs::msg::Twist> last_twist_;
    std::map<std::string, bool> is_decelerating_;
    std::map<std::string, int> decel_step_count_;
    std::map<std::string, rclcpp::Time> accel_start_time_;  // 基于时间的加速斜坡，替代计数方式
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_publisher_;

    rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr left_arm_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr left_arm_publisher_;
     
    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr right_arm_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr right_arm_publisher_;

    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr left_arm_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr right_arm_status_subscription_;
    std::map<std::string, int8_t> last_servo_status_;
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