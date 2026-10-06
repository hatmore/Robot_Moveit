/**
 * ================================================================================
 * 速度控制服务器合并节点 (Velocity Controllers Node)
 * ================================================================================
 *
 * 合并以下2个速度控制节点到单一节点:
 *   1. TcpVelocityToServo - TCP速度控制
 *   2. JointVelocityToServo - 关节速度控制
 *
 * 安全特性:
 *   - 关节位置限位检查：接近限位时自动减速
 *   - 平滑加速/减速：避免突变
 *   - 指令超时检测：超时自动停车
 *   - Servo状态监控：奇异点、碰撞等警告
 *
 * ================================================================================
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <planning_sdk_msgs/msg/tcp_velocity_once.hpp>
#include <planning_sdk_msgs/msg/joint_velocity_once.hpp>
#include <planning_sdk_msgs/msg/joint_limits_array.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int8.hpp>

#include "basic_control_topic/shared_resources.hpp"

#include <map>
#include <mutex>
#include <string>
#include <atomic>
#include <vector>

namespace basic_control_topic
{

/**
 * 速度控制服务器合并节点
 */
class VelocityControllersNode : public rclcpp::Node
{
public:
    explicit VelocityControllersNode(const SharedResources::Ptr& shared_resources,
                                      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~VelocityControllersNode() = default;

private:
    SharedResources::Ptr shared_resources_;

    // ========== 关节位置限位（共享） ==========
    rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::map<std::string, double> joint_min_positions_;
    std::map<std::string, double> joint_max_positions_;
    std::map<std::string, double> current_joint_positions_;
    std::mutex joint_state_mutex_;
    std::atomic<bool> joint_states_received_{false};

    void jointLimitsCallback(const planning_sdk_msgs::msg::JointLimitsArray::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    double computePositionLimitScale();
    double clampVelocityForPosition(const std::string& joint_name, double velocity);
    void initializeJointPositions();
    void waitForJointStates(double timeout_sec = 5.0);

    // ========== TCP 速度控制 ==========
    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr left_tcp_vel_sub_;
    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr right_tcp_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr left_tcp_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr right_tcp_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr left_servo_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr right_servo_status_sub_;

    double max_linear_velocity_ = 2.0;
    double max_angular_velocity_ = 2.5;
    bool verbose_logging_ = true;

    bool left_arm_publisher_connected_ = false;
    bool right_arm_publisher_connected_ = false;

    // 平滑加速/减速
    double decel_duration_ = 0.3;
    int decel_steps_ = 10;
    double command_timeout_ = 0.15;
    std::map<std::string, rclcpp::Time> last_tcp_cmd_time_;
    std::map<std::string, geometry_msgs::msg::Twist> last_tcp_twist_;
    std::map<std::string, bool> is_tcp_decelerating_;
    std::map<std::string, int> tcp_decel_step_count_;
    std::map<std::string, int> tcp_accel_step_count_;
    std::map<std::string, int> tcp_repeated_count_;
    std::map<std::string, int> singularity_warning_count_;
    std::map<std::string, int8_t> last_servo_status_;

    // 100Hz 定时发布缓存（各臂独立，mutex 保护）
    geometry_msgs::msg::TwistStamped pending_left_tcp_twist_;
    geometry_msgs::msg::TwistStamped pending_right_tcp_twist_;
    std::mutex pending_tcp_mutex_left_;
    std::mutex pending_tcp_mutex_right_;

    void leftTcpVelocityCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg);
    void rightTcpVelocityCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg);
    void processTcpVelocityCommand(
        const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg,
        const std::string& arm_side);
    void servoStatusCallback(const std_msgs::msg::Int8::SharedPtr msg, const std::string& arm_side);
    void checkTcpVelConnections();
    void publishTimerCallback();  // 合并 decel + publish，100Hz

    rclcpp::TimerBase::SharedPtr tcp_vel_connection_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // ========== 关节速度控制 ==========
    rclcpp::Subscription<planning_sdk_msgs::msg::JointVelocityOnce>::SharedPtr joint_vel_sub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr left_joint_vel_pub_;
    rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr right_joint_vel_pub_;

    double velocity_scaling_factor_ = 0.8;
    bool left_arm_joint_publisher_connected_ = false;
    bool right_arm_joint_publisher_connected_ = false;

    std::vector<std::string> left_joint_order_;
    std::vector<std::string> right_joint_order_;
    std::map<std::string, double> joint_velocities_;
    std::map<std::string, double> joint_init_velocities_;
    std::map<std::string, double> joint_max_velocities_;

    void jointVelocityCallback(const planning_sdk_msgs::msg::JointVelocityOnce::SharedPtr msg);
    void checkJointVelConnections();
    void initializeJointConfiguration();
    void loadJointLimitsFromParams();

    rclcpp::TimerBase::SharedPtr joint_vel_connection_timer_;

    // ========== 诊断 ==========
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diagnostic_pub_;
    void publishDiagnostic(const std::string& message);
};

}  // namespace basic_control_topic
