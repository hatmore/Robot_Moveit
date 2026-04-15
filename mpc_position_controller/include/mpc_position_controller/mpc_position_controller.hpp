#pragma once

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>

#include "mpc_position_controller/mpc_solver.hpp"

namespace mpc_position_controller
{

class MPCPositionController : public controller_interface::ControllerInterface
{
public:
  using FollowJTAction = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJTAction>;

  MPCPositionController() = default;

  // ---- Lifecycle ----
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State& previous_state) override;

  // ---- Interface configuration ----
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  // ---- Real-time update ----
  controller_interface::return_type update(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // ---------- Joint configuration ----------
  std::vector<std::string> joint_names_;
  size_t num_joints_ = 0;

  // ---------- MPC ----------
  std::unique_ptr<MPCSolver> solver_;
  MPCConfig mpc_config_;

  // ---------- Trajectory input ----------
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;

  struct ActiveTrajectory
  {
    trajectory_msgs::msg::JointTrajectory traj;
    rclcpp::Time start_time;
    bool active = false;
  };
  realtime_tools::RealtimeBuffer<ActiveTrajectory> active_traj_buf_;

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);

  // Reorder incoming trajectory joints to match joint_names_ order
  bool reorderTrajectory(trajectory_msgs::msg::JointTrajectory& traj) const;

  // ---------- FollowJointTrajectory action server ----------
  rclcpp_action::Server<FollowJTAction>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleFJT> active_goal_handle_;
  std::mutex goal_mutex_;
  std::atomic<bool> goal_active_{false};

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const FollowJTAction::Goal> goal);

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleFJT> goal_handle);

  void handleAccepted(const std::shared_ptr<GoalHandleFJT> goal_handle);

  // ---------- Reference building ----------
  MPCReference buildReference(const rclcpp::Time& current_time);

  // Linear interpolation between two trajectory points
  void interpolatePoint(
    const trajectory_msgs::msg::JointTrajectoryPoint& p0,
    const trajectory_msgs::msg::JointTrajectoryPoint& p1,
    double t0, double t1, double t_query,
    Eigen::VectorXd& q_out, Eigen::VectorXd& v_out) const;

  // ---------- State ----------
  Eigen::VectorXd current_q_;
  Eigen::VectorXd current_v_;
  Eigen::VectorXd cmd_q_;
  Eigen::VectorXd hold_position_;
  bool is_holding_ = true;
};

}  // namespace mpc_position_controller
