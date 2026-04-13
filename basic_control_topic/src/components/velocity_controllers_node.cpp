/**
 * ================================================================================
 * 速度控制服务器合并节点实现
 * ================================================================================
 */

#include "basic_control_topic/components/velocity_controllers_node.hpp"
#include <algorithm>
#include <cmath>
#include <thread>

namespace basic_control_topic
{

VelocityControllersNode::VelocityControllersNode(
    const SharedResources::Ptr& shared_resources,
    const rclcpp::NodeOptions& options)
    : Node("velocity_controllers_node", options)
    , shared_resources_(shared_resources)
{
    // 声明参数
    this->declare_parameter<bool>("enable_left_arm", true);
    this->declare_parameter<bool>("enable_right_arm", true);
    this->declare_parameter<double>("max_linear_velocity", 2.0);
    this->declare_parameter<double>("max_angular_velocity", 2.5);
    this->declare_parameter<double>("velocity_scaling_factor", 0.8);
    this->declare_parameter<bool>("verbose_logging", true);
    this->declare_parameter<double>("decel_duration", 0.3);
    this->declare_parameter<int>("decel_steps", 50);
    this->declare_parameter<double>("command_timeout", 0.15);

    max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
    max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
    velocity_scaling_factor_ = this->get_parameter("velocity_scaling_factor").as_double();
    verbose_logging_ = this->get_parameter("verbose_logging").as_bool();
    decel_duration_ = this->get_parameter("decel_duration").as_double();
    decel_steps_ = this->get_parameter("decel_steps").as_int();
    command_timeout_ = this->get_parameter("command_timeout").as_double();

    bool enable_left_arm = this->get_parameter("enable_left_arm").as_bool();
    bool enable_right_arm = this->get_parameter("enable_right_arm").as_bool();

    // ========== 初始化关节位置限位 ==========
    initializeJointPositions();

    // 订阅动态关节限位（transient_local，与 server 端 QoS 匹配）
    auto limits_qos = rclcpp::QoS(1).transient_local();
    joint_limits_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointLimitsArray>(
        "/algorithm/joint_limits/current", limits_qos,
        std::bind(&VelocityControllersNode::jointLimitsCallback, this, std::placeholders::_1));

    // 订阅关节状态（用于位置限位）
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&VelocityControllersNode::jointStateCallback, this, std::placeholders::_1));

    // ========== TCP 速度控制 ==========
    if (enable_left_arm)
    {
        left_tcp_vel_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
            "/algorithm/grasp_planning/move/left_arm/tcp_velocity_once",
            10,
            std::bind(&VelocityControllersNode::leftTcpVelocityCallback, this, std::placeholders::_1));

        // QoS 与 servo 节点的 SensorDataQoS 订阅匹配
        auto tcp_qos = rclcpp::SensorDataQoS();
        left_tcp_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/left_arm_servo_node/delta_twist_cmds", tcp_qos);

        left_servo_status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/left_arm_servo_node/status", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                servoStatusCallback(msg, "left");
            });

        RCLCPP_INFO(this->get_logger(), "左臂TCP速度控制已启用");
    }

    if (enable_right_arm)
    {
        right_tcp_vel_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
            "/algorithm/grasp_planning/move/right_arm/tcp_velocity_once",
            10,
            std::bind(&VelocityControllersNode::rightTcpVelocityCallback, this, std::placeholders::_1));

        auto tcp_qos = rclcpp::SensorDataQoS();
        right_tcp_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/right_arm_servo_node/delta_twist_cmds", tcp_qos);

        right_servo_status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/right_arm_servo_node/status", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                servoStatusCallback(msg, "right");
            });

        RCLCPP_INFO(this->get_logger(), "右臂TCP速度控制已启用");
    }

    // 初始化 TCP 减速相关状态
    tcp_repeated_count_["left"] = 0;
    tcp_repeated_count_["right"] = 0;
    singularity_warning_count_["left"] = 0;
    singularity_warning_count_["right"] = 0;
    is_tcp_decelerating_["left"] = false;
    is_tcp_decelerating_["right"] = false;
    tcp_decel_step_count_["left"] = 0;
    tcp_decel_step_count_["right"] = 0;
    tcp_accel_step_count_["left"] = 0;
    tcp_accel_step_count_["right"] = 0;

    tcp_vel_connection_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&VelocityControllersNode::checkTcpVelConnections, this));

    // 100Hz 定时发布（含减速逻辑）
    publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&VelocityControllersNode::publishTimerCallback, this));

    // ========== 关节速度控制 ==========
    initializeJointConfiguration();
    loadJointLimitsFromParams();

    joint_vel_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointVelocityOnce>(
        "/algorithm/grasp_planning/move/joint_velocity_once",
        10,
        std::bind(&VelocityControllersNode::jointVelocityCallback, this, std::placeholders::_1));

    // QoS 与 servo 节点的 SensorDataQoS 订阅匹配
    auto joint_qos = rclcpp::SensorDataQoS();

    if (enable_left_arm)
    {
        left_joint_vel_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/left_arm_servo_node/delta_joint_cmds", joint_qos);
        RCLCPP_INFO(this->get_logger(), "左臂关节速度控制已启用");
    }

    if (enable_right_arm)
    {
        right_joint_vel_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
            "/right_arm_servo_node/delta_joint_cmds", joint_qos);
        RCLCPP_INFO(this->get_logger(), "右臂关节速度控制已启用");
    }

    joint_vel_connection_timer_ = this->create_wall_timer(
        std::chrono::seconds(3),
        std::bind(&VelocityControllersNode::checkJointVelConnections, this));

    // ========== 诊断发布 ==========
    diagnostic_pub_ = this->create_publisher<std_msgs::msg::String>("/velocity_diagnostics", 10);

    // 等待获取关节状态
    RCLCPP_INFO(this->get_logger(), "等待获取关节状态...");
    waitForJointStates();

    RCLCPP_INFO(this->get_logger(), "Velocity Controllers Node initialized");
    RCLCPP_INFO(this->get_logger(), "  - Max Linear Velocity: %.3f m/s", max_linear_velocity_);
    RCLCPP_INFO(this->get_logger(), "  - Max Angular Velocity: %.3f rad/s", max_angular_velocity_);
    RCLCPP_INFO(this->get_logger(), "  - Velocity Scaling Factor: %.2f", velocity_scaling_factor_);
    RCLCPP_INFO(this->get_logger(), "  - Decel Steps: %d", decel_steps_);
    RCLCPP_INFO(this->get_logger(), "  - Command Timeout: %.3f s", command_timeout_);
}

// ============================================================================
// 关节位置限位
// ============================================================================
void VelocityControllersNode::initializeJointPositions()
{
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
}

void VelocityControllersNode::jointLimitsCallback(const planning_sdk_msgs::msg::JointLimitsArray::SharedPtr msg)
{
    for (const auto& jl : msg->limits) {
        if (joint_min_positions_.count(jl.joint_name)) {
            joint_min_positions_[jl.joint_name] = jl.lower_limit;
            joint_max_positions_[jl.joint_name] = jl.upper_limit;
        }
        if (joint_max_velocities_.count(jl.joint_name) && jl.velocity_limit > 0.0) {
            joint_max_velocities_[jl.joint_name] = jl.velocity_limit;
        }
    }
    RCLCPP_DEBUG(this->get_logger(), "Updated joint limits (%zu joints)", msg->limits.size());
}

void VelocityControllersNode::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    bool has_valid_data = false;
    for (size_t i = 0; i < msg->name.size(); ++i) {
        auto it = current_joint_positions_.find(msg->name[i]);
        if (it != current_joint_positions_.end()) {
            it->second = msg->position[i];
            has_valid_data = true;
        }
    }
    if (has_valid_data) {
        joint_states_received_ = true;
    }
}

double VelocityControllersNode::computePositionLimitScale()
{
    constexpr double kBuffer = 0.05;
    double scale = 1.0;
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (const auto& [name, pos] : current_joint_positions_) {
        double min_pos = joint_min_positions_[name];
        double max_pos = joint_max_positions_[name];
        double dist_upper = max_pos - pos;
        double dist_lower = pos - min_pos;
        double s = 1.0;
        if (dist_upper < kBuffer) s = std::min(s, dist_upper / kBuffer);
        if (dist_lower < kBuffer) s = std::min(s, dist_lower / kBuffer);
        s = std::max(s, 0.0);
        scale = std::min(scale, s);
    }
    return scale;
}

double VelocityControllersNode::clampVelocityForPosition(const std::string& joint_name, double velocity)
{
    constexpr double kBuffer = 0.05;
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    auto it = current_joint_positions_.find(joint_name);
    if (it == current_joint_positions_.end()) return velocity;
    double pos = it->second;
    double min_pos = joint_min_positions_[joint_name];
    double max_pos = joint_max_positions_[joint_name];

    if (velocity > 0.0) {
        double dist = max_pos - pos;
        if (dist <= 0.0) return 0.0;
        if (dist < kBuffer) return velocity * (dist / kBuffer);
    } else if (velocity < 0.0) {
        double dist = pos - min_pos;
        if (dist <= 0.0) return 0.0;
        if (dist < kBuffer) return velocity * (dist / kBuffer);
    }
    return velocity;
}

void VelocityControllersNode::waitForJointStates(double timeout_sec)
{
    auto start_time = this->now();
    rclcpp::Rate rate(10);

    while (rclcpp::ok()) {
        if (joint_states_received_) {
            return;
        }
        auto elapsed = (this->now() - start_time).seconds();
        if (elapsed > timeout_sec) {
            RCLCPP_WARN(this->get_logger(), "等待关节状态超时 (%.1f秒)，将继续使用默认值", timeout_sec);
            joint_states_received_ = true;
            return;
        }
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }
}

// ============================================================================
// TCP 速度控制
// ============================================================================
void VelocityControllersNode::leftTcpVelocityCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg)
{
    if (shared_resources_ && shared_resources_->isEmergencyStopActive()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Emergency stop active, rejecting left TCP velocity command");
        return;
    }
    if (!left_arm_publisher_connected_ || !left_tcp_vel_pub_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "左臂Servo节点未连接");
        return;
    }
    if (tcp_accel_step_count_["left"] < decel_steps_) {
        tcp_accel_step_count_["left"]++;
    }
    processTcpVelocityCommand(msg, "left");
}

void VelocityControllersNode::rightTcpVelocityCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg)
{
    if (shared_resources_ && shared_resources_->isEmergencyStopActive()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Emergency stop active, rejecting right TCP velocity command");
        return;
    }
    if (!right_arm_publisher_connected_ || !right_tcp_vel_pub_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "右臂Servo节点未连接");
        return;
    }
    if (tcp_accel_step_count_["right"] < decel_steps_) {
        tcp_accel_step_count_["right"]++;
    }
    processTcpVelocityCommand(msg, "right");
}

void VelocityControllersNode::processTcpVelocityCommand(
    const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg,
    const std::string& arm_side)
{
    if (singularity_warning_count_[arm_side] > 5) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                              "%s机械臂持续处于奇异点附近，请手动调整机械臂姿态", arm_side.c_str());
        return;
    }

    bool was_idle = (last_tcp_cmd_time_.find(arm_side) == last_tcp_cmd_time_.end()) ||
                    (is_tcp_decelerating_[arm_side] && tcp_decel_step_count_[arm_side] >= decel_steps_);
    if (was_idle) {
        tcp_accel_step_count_[arm_side] = 0;
        is_tcp_decelerating_[arm_side] = false;
        tcp_decel_step_count_[arm_side] = 0;
    }

    double accel_ratio = (tcp_accel_step_count_[arm_side] < decel_steps_)
        ? static_cast<double>(tcp_accel_step_count_[arm_side]) / decel_steps_ : 1.0;

    constexpr double kVelocityScale = 1.0 / 100.0;
    auto clamp = [](double v) { return std::max(-100.0, std::min(100.0, v)); };

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = "base_link";
    twist.twist.linear.x  = clamp(msg->velocity.linear.x)  * kVelocityScale * max_linear_velocity_  * accel_ratio;
    twist.twist.linear.y  = clamp(msg->velocity.linear.y)  * kVelocityScale * max_linear_velocity_  * accel_ratio;
    twist.twist.linear.z  = clamp(msg->velocity.linear.z)  * kVelocityScale * max_linear_velocity_  * accel_ratio;
    twist.twist.angular.x = clamp(msg->velocity.angular.x) * kVelocityScale * max_angular_velocity_ * accel_ratio;
    twist.twist.angular.y = clamp(msg->velocity.angular.y) * kVelocityScale * max_angular_velocity_ * accel_ratio;
    twist.twist.angular.z = clamp(msg->velocity.angular.z) * kVelocityScale * max_angular_velocity_ * accel_ratio;

    double pos_scale = computePositionLimitScale();
    twist.twist.linear.x  *= pos_scale;
    twist.twist.linear.y  *= pos_scale;
    twist.twist.linear.z  *= pos_scale;
    twist.twist.angular.x *= pos_scale;
    twist.twist.angular.y *= pos_scale;
    twist.twist.angular.z *= pos_scale;

    // 写入对应臂缓存，定时器负责发布
    if (arm_side == "left") {
        std::lock_guard<std::mutex> lock(pending_tcp_mutex_left_);
        pending_left_tcp_twist_ = twist;
    } else {
        std::lock_guard<std::mutex> lock(pending_tcp_mutex_right_);
        pending_right_tcp_twist_ = twist;
    }

    last_tcp_cmd_time_[arm_side] = this->now();
    last_tcp_twist_[arm_side] = msg->velocity;
    is_tcp_decelerating_[arm_side] = false;
    tcp_decel_step_count_[arm_side] = 0;
}

void VelocityControllersNode::servoStatusCallback(const std_msgs::msg::Int8::SharedPtr msg, const std::string& arm_side)
{
    int8_t status = msg->data;
    auto& last = last_servo_status_[arm_side];
    if (status == last && status == 0) {
        return;
    }
    last = status;

    switch (status) {
        case 0:
            RCLCPP_INFO(this->get_logger(), "[%s臂Servo] 状态恢复正常", arm_side.c_str());
            singularity_warning_count_[arm_side] = 0;
            break;
        case 1:
            RCLCPP_WARN(this->get_logger(), "[%s臂Servo] 接近奇异点，正在减速", arm_side.c_str());
            singularity_warning_count_[arm_side]++;
            break;
        case 2:
            RCLCPP_ERROR(this->get_logger(), "[%s臂Servo] 非常接近奇异点，硬停止!", arm_side.c_str());
            singularity_warning_count_[arm_side]++;
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

void VelocityControllersNode::checkTcpVelConnections()
{
    auto check_conn = [this](const std::string& arm_side,
                             const rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr& pub,
                             bool& connected) {
        if (pub) {
            auto subs = pub->get_subscription_count();
            if (subs > 0 && !connected) {
                connected = true;
                RCLCPP_INFO(this->get_logger(), "%s臂TCP Servo节点已连接", arm_side.c_str());
                publishDiagnostic(arm_side + "臂TCP Servo节点已连接");
            } else if (subs == 0 && connected) {
                connected = false;
                RCLCPP_WARN(this->get_logger(), "%s臂TCP Servo节点连接丢失", arm_side.c_str());
                publishDiagnostic(arm_side + "臂TCP Servo节点连接丢失");
            }
        }
    };

    check_conn("左", left_tcp_vel_pub_, left_arm_publisher_connected_);
    check_conn("右", right_tcp_vel_pub_, right_arm_publisher_connected_);
}

void VelocityControllersNode::publishTimerCallback()
{
    auto now = this->now();
    constexpr double kVelocityScale = 1.0 / 100.0;

    // ---- 左臂 ----
    if (left_tcp_vel_pub_ && left_arm_publisher_connected_) {
        if (last_tcp_cmd_time_.count("left")) {
            double elapsed = (now - last_tcp_cmd_time_["left"]).seconds();
            if (elapsed > command_timeout_ && !is_tcp_decelerating_["left"] && tcp_decel_step_count_["left"] == 0) {
                is_tcp_decelerating_["left"] = true;
                RCLCPP_INFO(this->get_logger(), "左臂指令超时，开始平滑停车");
            }
        }

        geometry_msgs::msg::TwistStamped out;
        out.header.stamp = now;
        out.header.frame_id = "base_link";

        if (is_tcp_decelerating_["left"] && tcp_decel_step_count_["left"] < decel_steps_) {
            tcp_decel_step_count_["left"]++;
            double ratio = 1.0 - static_cast<double>(tcp_decel_step_count_["left"]) / decel_steps_;
            out.twist.linear.x  = last_tcp_twist_["left"].linear.x  * kVelocityScale * max_linear_velocity_  * ratio;
            out.twist.linear.y  = last_tcp_twist_["left"].linear.y  * kVelocityScale * max_linear_velocity_  * ratio;
            out.twist.linear.z  = last_tcp_twist_["left"].linear.z  * kVelocityScale * max_linear_velocity_  * ratio;
            out.twist.angular.x = last_tcp_twist_["left"].angular.x * kVelocityScale * max_angular_velocity_ * ratio;
            out.twist.angular.y = last_tcp_twist_["left"].angular.y * kVelocityScale * max_angular_velocity_ * ratio;
            out.twist.angular.z = last_tcp_twist_["left"].angular.z * kVelocityScale * max_angular_velocity_ * ratio;
            left_tcp_vel_pub_->publish(out);
        } else if (is_tcp_decelerating_["left"] && tcp_decel_step_count_["left"] >= decel_steps_) {
            is_tcp_decelerating_["left"] = false;
            last_tcp_cmd_time_.erase("left");  // 清除，防止定时器继续发缓存
        } else if (!is_tcp_decelerating_["left"] && last_tcp_cmd_time_.count("left")) {
            std::lock_guard<std::mutex> lock(pending_tcp_mutex_left_);
            pending_left_tcp_twist_.header.stamp = now;
            left_tcp_vel_pub_->publish(pending_left_tcp_twist_);
        }
    }

    // ---- 右臂 ----
    if (right_tcp_vel_pub_ && right_arm_publisher_connected_) {
        if (last_tcp_cmd_time_.count("right")) {
            double elapsed = (now - last_tcp_cmd_time_["right"]).seconds();
            if (elapsed > command_timeout_ && !is_tcp_decelerating_["right"] && tcp_decel_step_count_["right"] == 0) {
                is_tcp_decelerating_["right"] = true;
                RCLCPP_INFO(this->get_logger(), "右臂指令超时，开始平滑停车");
            }
        }

        geometry_msgs::msg::TwistStamped out;
        out.header.stamp = now;
        out.header.frame_id = "base_link";

        if (is_tcp_decelerating_["right"] && tcp_decel_step_count_["right"] < decel_steps_) {
            tcp_decel_step_count_["right"]++;
            double ratio = 1.0 - static_cast<double>(tcp_decel_step_count_["right"]) / decel_steps_;
            out.twist.linear.x  = last_tcp_twist_["right"].linear.x  * kVelocityScale * max_linear_velocity_  * ratio;
            out.twist.linear.y  = last_tcp_twist_["right"].linear.y  * kVelocityScale * max_linear_velocity_  * ratio;
            out.twist.linear.z  = last_tcp_twist_["right"].linear.z  * kVelocityScale * max_linear_velocity_  * ratio;
            out.twist.angular.x = last_tcp_twist_["right"].angular.x * kVelocityScale * max_angular_velocity_ * ratio;
            out.twist.angular.y = last_tcp_twist_["right"].angular.y * kVelocityScale * max_angular_velocity_ * ratio;
            out.twist.angular.z = last_tcp_twist_["right"].angular.z * kVelocityScale * max_angular_velocity_ * ratio;
            right_tcp_vel_pub_->publish(out);
        } else if (is_tcp_decelerating_["right"] && tcp_decel_step_count_["right"] >= decel_steps_) {
            is_tcp_decelerating_["right"] = false;
            last_tcp_cmd_time_.erase("right");  // 清除，防止定时器继续发缓存
        } else if (!is_tcp_decelerating_["right"] && last_tcp_cmd_time_.count("right")) {
            std::lock_guard<std::mutex> lock(pending_tcp_mutex_right_);
            pending_right_tcp_twist_.header.stamp = now;
            right_tcp_vel_pub_->publish(pending_right_tcp_twist_);
        }
    }
}

// ============================================================================
// 关节速度控制
// ============================================================================
void VelocityControllersNode::initializeJointConfiguration()
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
            joint_max_velocities_[joint_name] = 3.0;
            joint_min_positions_[joint_name] = -3.14159;
            joint_max_positions_[joint_name] = 3.14159;
            current_joint_positions_[joint_name] = 0.0;
        }
    };

    initializeJoints(left_joint_order_);
    initializeJoints(right_joint_order_);
    joint_init_velocities_ = joint_velocities_;
}

void VelocityControllersNode::loadJointLimitsFromParams()
{
    for (auto& [joint_name, max_vel] : joint_max_velocities_) {
        std::string param_name = "robot_description_planning.joint_limits." + joint_name + ".max_velocity";
        this->declare_parameter<double>(param_name, max_vel);
        double val = this->get_parameter(param_name).as_double();
        if (val > 0.0) {
            max_vel = val;
        }
    }
}

void VelocityControllersNode::jointVelocityCallback(const planning_sdk_msgs::msg::JointVelocityOnce::SharedPtr msg)
{
    // 急停检查：拒绝处理命令
    if (shared_resources_ && shared_resources_->isEmergencyStopActive()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Emergency stop active, rejecting joint velocity command");
        return;
    }

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
        if (!left_arm_joint_publisher_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "左臂关节Servo节点未连接");
            return;
        }
    } else if (joint_name.find("right") != std::string::npos) {
        arm_side = "right";
        if (!right_arm_joint_publisher_connected_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                               "右臂关节Servo节点未连接");
            return;
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "未知的关节名称: %s", joint_name.c_str());
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

    // 应用位置限位
    actual_velocity = clampVelocityForPosition(joint_name, actual_velocity);
    joint_velocities_[joint_name] = actual_velocity;

    // 发布到对应的臂
    if (arm_side == "left" && left_arm_joint_publisher_connected_ && left_joint_vel_pub_) {
        auto left_msg = std::make_unique<control_msgs::msg::JointJog>();
        left_msg->header.stamp = this->now();
        left_msg->header.frame_id = "base_link";
        for (const auto& jname : left_joint_order_) {
            left_msg->joint_names.push_back(jname);
            left_msg->velocities.push_back(joint_velocities_[jname]);
        }
        left_msg->duration = 0.01;
        left_joint_vel_pub_->publish(std::move(left_msg));
    } else if (arm_side == "right" && right_arm_joint_publisher_connected_ && right_joint_vel_pub_) {
        auto right_msg = std::make_unique<control_msgs::msg::JointJog>();
        right_msg->header.stamp = this->now();
        right_msg->header.frame_id = "base_link";
        for (const auto& jname : right_joint_order_) {
            right_msg->joint_names.push_back(jname);
            right_msg->velocities.push_back(joint_velocities_[jname]);
        }
        right_msg->duration = 0.01;
        right_joint_vel_pub_->publish(std::move(right_msg));
    }
}

void VelocityControllersNode::checkJointVelConnections()
{
    auto check_conn = [this](const std::string& arm_side,
                             const rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr& pub,
                             bool& connected) {
        if (pub) {
            auto subs = pub->get_subscription_count();
            if (subs > 0 && !connected) {
                connected = true;
                RCLCPP_INFO(this->get_logger(), "%s臂关节Servo节点已连接", arm_side.c_str());
            } else if (subs == 0 && connected) {
                connected = false;
                RCLCPP_WARN(this->get_logger(), "%s臂关节Servo节点连接丢失", arm_side.c_str());
            }
        }
    };

    check_conn("左", left_joint_vel_pub_, left_arm_joint_publisher_connected_);
    check_conn("右", right_joint_vel_pub_, right_arm_joint_publisher_connected_);
}

// ============================================================================
// 诊断
// ============================================================================
void VelocityControllersNode::publishDiagnostic(const std::string& message)
{
    auto msg = std_msgs::msg::String();
    msg.data = message;
    diagnostic_pub_->publish(msg);
}

}  // namespace basic_control_topic
