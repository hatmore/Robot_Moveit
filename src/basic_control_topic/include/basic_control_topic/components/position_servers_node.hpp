/**
 * ================================================================================
 * 位置控制服务器合并节点 (Position Servers Node)
 * ================================================================================
 *
 * 合并以下4个位置控制 Action Servers 到单一节点:
 *   1. JointPositionServer - 关节位置控制
 *   2. TcpPositionServer - TCP位置控制
 *   3. JointPositionDeltaServer - 关节增量控制
 *   4. TcpPositionDeltaServer - TCP增量控制
 *
 * 优势:
 *   - 共享关节限位订阅
 *   - 共享关节状态监控 (JointStateMonitor)
 *   - 减少进程数量
 *
 * ================================================================================
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <planning_sdk_msgs/action/joint_position.hpp>
#include <planning_sdk_msgs/action/tcp_position.hpp>
#include <planning_sdk_msgs/action/joint_position_delta.hpp>
#include <planning_sdk_msgs/action/tcp_position_delta.hpp>
#include <planning_sdk_msgs/msg/joint_limits_array.hpp>
#include <planning_sdk_msgs/msg/tcp_pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include "basic_control_topic/shared_resources.hpp"
#include "basic_control_topic/joint_state_monitor.hpp"

#include <unordered_map>
#include <mutex>
#include <memory>
#include <thread>
#include <functional>

namespace basic_control_topic
{

// 类型别名
using JointPosition = planning_sdk_msgs::action::JointPosition;
using TcpPosition = planning_sdk_msgs::action::TcpPosition;
using JointPositionDelta = planning_sdk_msgs::action::JointPositionDelta;
using TcpPositionDelta = planning_sdk_msgs::action::TcpPositionDelta;

using GoalHandleJointPosition = rclcpp_action::ServerGoalHandle<JointPosition>;
using GoalHandleTcpPosition = rclcpp_action::ServerGoalHandle<TcpPosition>;
using GoalHandleJointPositionDelta = rclcpp_action::ServerGoalHandle<JointPositionDelta>;
using GoalHandleTcpPositionDelta = rclcpp_action::ServerGoalHandle<TcpPositionDelta>;

using GoalUUID = rclcpp_action::GoalUUID;

/**
 * GoalUUID 哈希函数，用于 unordered_map
 */
struct GoalUUIDHash
{
    size_t operator()(const GoalUUID& uuid) const
    {
        return std::hash<std::string>()(std::string(uuid.begin(), uuid.end()));
    }
};

/**
 * 位置控制服务器合并节点
 *
 * 提供所有位置控制相关的 Action Servers
 */
class PositionServersNode : public rclcpp::Node
{
public:
    explicit PositionServersNode(const SharedResources::Ptr& shared_resources,
                                  const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~PositionServersNode();

private:
    // ========== 共享资源 ==========
    SharedResources::Ptr shared_resources_;
    JointStateMonitor js_monitor_;

    // ========== MoveIt 持久化资源 ==========
    void initializeMoveIt();
    void cleanupMoveIt();
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> getMoveGroup(const std::string& arm_type);

    std::shared_ptr<rclcpp::Node> moveit_node_;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> moveit_executor_;
    std::shared_ptr<std::thread> moveit_thread_;

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_group_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_group_;
    std::mutex moveit_mutex_;

    // ========== 关节位置控制 ==========
    rclcpp_action::Server<JointPosition>::SharedPtr joint_position_server_;

    rclcpp_action::GoalResponse handleJointPositionGoal(
        const GoalUUID& uuid,
        std::shared_ptr<const JointPosition::Goal> goal);

    rclcpp_action::CancelResponse handleJointPositionCancel(
        const std::shared_ptr<GoalHandleJointPosition> goal_handle);

    void handleJointPositionAccepted(const std::shared_ptr<GoalHandleJointPosition> goal_handle);

    void executeJointPosition(const std::shared_ptr<GoalHandleJointPosition> goal_handle);

    // ========== TCP位置控制 ==========
    rclcpp_action::Server<TcpPosition>::SharedPtr left_tcp_position_server_;
    rclcpp_action::Server<TcpPosition>::SharedPtr right_tcp_position_server_;
    std::unordered_map<GoalUUID, std::string, GoalUUIDHash> tcp_position_goal_arm_map_;
    std::mutex tcp_position_map_mutex_;

    rclcpp_action::GoalResponse handleTcpPositionGoal(
        const GoalUUID& uuid,
        std::shared_ptr<const TcpPosition::Goal> goal,
        const std::string& arm_type);

    rclcpp_action::CancelResponse handleTcpPositionCancel(
        const std::shared_ptr<GoalHandleTcpPosition> goal_handle);

    void handleTcpPositionAccepted(const std::shared_ptr<GoalHandleTcpPosition> goal_handle);

    void executeTcpPosition(const std::shared_ptr<GoalHandleTcpPosition> goal_handle);

    // ========== 关节增量控制 ==========
    rclcpp_action::Server<JointPositionDelta>::SharedPtr joint_position_delta_server_;
    std::unordered_map<std::string, double> current_joint_pos_map_;
    bool joint_state_received_ = false;
    std::mutex joint_state_mutex_;
    std::mutex joint_delta_execute_mutex_;

    rclcpp_action::GoalResponse handleJointPositionDeltaGoal(
        const GoalUUID& uuid,
        std::shared_ptr<const JointPositionDelta::Goal> goal);

    rclcpp_action::CancelResponse handleJointPositionDeltaCancel(
        const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle);

    void handleJointPositionDeltaAccepted(const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle);

    void executeJointPositionDelta(const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle);

    // ========== TCP增量控制 ==========
    rclcpp_action::Server<TcpPositionDelta>::SharedPtr left_tcp_position_delta_server_;
    rclcpp_action::Server<TcpPositionDelta>::SharedPtr right_tcp_position_delta_server_;
    std::unordered_map<GoalUUID, std::string, GoalUUIDHash> tcp_delta_goal_arm_map_;
    std::mutex tcp_delta_map_mutex_;
    std::mutex tcp_delta_execute_mutex_;

    rclcpp::Subscription<planning_sdk_msgs::msg::TcpPose>::SharedPtr left_tcp_pose_sub_;
    rclcpp::Subscription<planning_sdk_msgs::msg::TcpPose>::SharedPtr right_tcp_pose_sub_;
    planning_sdk_msgs::msg::TcpPose latest_left_tcp_pose_;
    planning_sdk_msgs::msg::TcpPose latest_right_tcp_pose_;
    std::mutex left_pose_mutex_;
    std::mutex right_pose_mutex_;
    bool left_pose_received_ = false;
    bool right_pose_received_ = false;
    std::string right_end_effector_link_ = "right_gripper_base_link";
    std::string left_end_effector_link_ = "left_gripper_base_link";

    rclcpp_action::GoalResponse handleTcpPositionDeltaGoal(
        const GoalUUID& uuid,
        std::shared_ptr<const TcpPositionDelta::Goal> goal,
        const std::string& arm_type);

    rclcpp_action::CancelResponse handleTcpPositionDeltaCancel(
        const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle);

    void handleTcpPositionDeltaAccepted(const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle);

    void executeTcpPositionDelta(const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle);

    void leftTcpPoseCallback(const planning_sdk_msgs::msg::TcpPose::SharedPtr msg);
    void rightTcpPoseCallback(const planning_sdk_msgs::msg::TcpPose::SharedPtr msg);
    bool getLatestTcpPose(const std::string& arm_type, geometry_msgs::msg::PoseStamped& current_pose);

    // ========== 辅助方法 ==========
    bool identifyJointGroup(const std::vector<std::string>& joint_names, std::string& joint_group_name);
};

}  // namespace basic_control_topic