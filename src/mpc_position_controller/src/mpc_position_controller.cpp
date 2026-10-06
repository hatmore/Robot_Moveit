#include "mpc_position_controller/mpc_position_controller.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <algorithm>
#include <cmath>

namespace mpc_position_controller
{

// ============================================================================
// Lifecycle
// ============================================================================

controller_interface::CallbackReturn MPCPositionController::on_init()
{
  try {
    // Declare parameters (read in on_configure)
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<std::string>>("command_interfaces", {"position"});
    auto_declare<std::vector<std::string>>("state_interfaces", {"position", "velocity"});

    auto_declare<int>("mpc.horizon", 10);
    auto_declare<double>("mpc.q_weight", 100.0);
    auto_declare<double>("mpc.v_weight", 1.0);
    auto_declare<double>("mpc.r_weight", 0.01);
    auto_declare<double>("mpc.constraint_margin", 0.05);
    auto_declare<double>("mpc.terminal_cost_multiplier", 3.0);

    auto_declare<std::vector<double>>("limits.q_min", {});
    auto_declare<std::vector<double>>("limits.q_max", {});
    auto_declare<std::vector<double>>("limits.v_max", {});
    auto_declare<std::vector<double>>("limits.a_max", {});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init exception: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCPositionController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // --- Read joint names ---
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified");
    return controller_interface::CallbackReturn::ERROR;
  }
  num_joints_ = joint_names_.size();
  const int n = static_cast<int>(num_joints_);

  RCLCPP_INFO(get_node()->get_logger(), "Configuring MPC controller for %d joints", n);

  // --- Build MPCConfig ---
  mpc_config_.num_joints = n;
  mpc_config_.horizon = get_node()->get_parameter("mpc.horizon").as_int();
  // dt comes from controller_manager update_rate
  mpc_config_.dt = 1.0 / static_cast<double>(get_update_rate());

  double q_w = get_node()->get_parameter("mpc.q_weight").as_double();
  double v_w = get_node()->get_parameter("mpc.v_weight").as_double();
  double r_w = get_node()->get_parameter("mpc.r_weight").as_double();
  mpc_config_.q_weights = Eigen::VectorXd::Constant(n, q_w);
  mpc_config_.v_weights = Eigen::VectorXd::Constant(n, v_w);
  mpc_config_.r_weights = Eigen::VectorXd::Constant(n, r_w);
  mpc_config_.constraint_margin = get_node()->get_parameter("mpc.constraint_margin").as_double();
  mpc_config_.terminal_cost_multiplier =
      get_node()->get_parameter("mpc.terminal_cost_multiplier").as_double();

  // --- Joint limits ---
  auto read_limit = [&](const std::string& param, double default_val) -> Eigen::VectorXd {
    auto vec = get_node()->get_parameter(param).as_double_array();
    if (static_cast<int>(vec.size()) == n) {
      return Eigen::Map<Eigen::VectorXd>(vec.data(), n);
    }
    RCLCPP_WARN(get_node()->get_logger(),
        "%s size mismatch (%zu vs %d), using default %.2f", param.c_str(), vec.size(), n, default_val);
    return Eigen::VectorXd::Constant(n, default_val);
  };

  mpc_config_.q_min = read_limit("limits.q_min", -3.14159);
  mpc_config_.q_max = read_limit("limits.q_max",  3.14159);
  mpc_config_.v_max = read_limit("limits.v_max",  3.0);
  mpc_config_.a_max = read_limit("limits.a_max",  9.0);

  // --- Create solver ---
  solver_ = std::make_unique<MPCSolver>(mpc_config_);

  // --- Pre-allocate Eigen vectors ---
  current_q_.resize(n);
  current_v_.resize(n);
  cmd_q_.resize(n);
  hold_position_.resize(n);
  current_q_.setZero();
  current_v_.setZero();
  cmd_q_.setZero();
  hold_position_.setZero();

  // --- Trajectory topic subscription ---
  trajectory_sub_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "~/joint_trajectory", rclcpp::SystemDefaultsQoS(),
      std::bind(&MPCPositionController::trajectoryCallback, this, std::placeholders::_1));

  // --- FollowJointTrajectory action server ---
  action_server_ = rclcpp_action::create_server<FollowJTAction>(
      get_node(), std::string(get_node()->get_name()) + "/follow_joint_trajectory",
      std::bind(&MPCPositionController::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MPCPositionController::handleCancel, this, std::placeholders::_1),
      std::bind(&MPCPositionController::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(get_node()->get_logger(),
      "MPC controller configured: horizon=%d, dt=%.4f, q_w=%.1f, v_w=%.1f, r_w=%.3f",
      mpc_config_.horizon, mpc_config_.dt, q_w, v_w, r_w);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCPositionController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Read current positions as hold position
  for (size_t i = 0; i < num_joints_; ++i) {
    hold_position_(i) = state_interfaces_[2 * i].get_value();  // position
    current_q_(i) = hold_position_(i);
    current_v_(i) = state_interfaces_[2 * i + 1].get_value();  // velocity
  }
  is_holding_ = true;

  // Write current positions as initial command (prevent jump)
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(hold_position_(i));
  }

  RCLCPP_INFO(get_node()->get_logger(), "MPC controller activated, holding current position");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCPositionController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Clear trajectory
  ActiveTrajectory empty;
  active_traj_buf_.writeFromNonRT(empty);
  is_holding_ = true;
  goal_active_.store(false);

  RCLCPP_INFO(get_node()->get_logger(), "MPC controller deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCPositionController::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  solver_.reset();
  trajectory_sub_.reset();
  action_server_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MPCPositionController::on_error(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ============================================================================
// Interface Configuration
// ============================================================================

controller_interface::InterfaceConfiguration
MPCPositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    conf.names.push_back(joint + "/position");
  }
  return conf;
}

controller_interface::InterfaceConfiguration
MPCPositionController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& joint : joint_names_) {
    conf.names.push_back(joint + "/position");
    conf.names.push_back(joint + "/velocity");
  }
  return conf;
}

// ============================================================================
// Real-time update
// ============================================================================

controller_interface::return_type MPCPositionController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
  // 1. Read current state from hardware
  for (size_t i = 0; i < num_joints_; ++i) {
    current_q_(i) = state_interfaces_[2 * i].get_value();
    current_v_(i) = state_interfaces_[2 * i + 1].get_value();
  }

  // 2. Check for new trajectory from buffer
  const auto& active_traj = *active_traj_buf_.readFromRT();
  if (active_traj.active && !active_traj.traj.points.empty()) {
    is_holding_ = false;
  }

  // 3. Build reference
  MPCReference ref = buildReference(time);

  // 4. Solve MPC
  MPCState state{current_q_, current_v_};
  Eigen::VectorXd u_opt(num_joints_);

  bool ok = solver_->solve(state, ref, u_opt, cmd_q_);
  if (!ok) {
    // Fallback: hold current position
    cmd_q_ = current_q_;
  }

  // 5. Write position commands
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces_[i].set_value(cmd_q_(i));
  }

  // 6. Check if trajectory completed (for action server feedback)
  if (goal_active_.load() && !is_holding_) {
    const auto& traj = active_traj.traj;
    if (!traj.points.empty()) {
      double traj_duration = traj.points.back().time_from_start.sec +
                             traj.points.back().time_from_start.nanosec * 1e-9;
      double elapsed = (time - active_traj.start_time).seconds();

      if (elapsed >= traj_duration) {
        // Trajectory completed — switch to holding final position
        const auto& last_pt = traj.points.back();
        for (size_t i = 0; i < num_joints_ && i < last_pt.positions.size(); ++i) {
          hold_position_(i) = last_pt.positions[i];
        }
        is_holding_ = true;

        // Notify action server
        std::lock_guard<std::mutex> lock(goal_mutex_);
        if (active_goal_handle_) {
          auto result = std::make_shared<FollowJTAction::Result>();
          result->error_code = FollowJTAction::Result::SUCCESSFUL;
          active_goal_handle_->succeed(result);
          active_goal_handle_.reset();
          goal_active_.store(false);
          RCLCPP_INFO(get_node()->get_logger(), "Trajectory completed successfully");
        }
      }
    }
  }

  return controller_interface::return_type::OK;
}

// ============================================================================
// Trajectory handling
// ============================================================================

void MPCPositionController::trajectoryCallback(
    const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
{
  if (msg->points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Received empty trajectory, ignoring");
    return;
  }

  auto traj = *msg;
  if (!reorderTrajectory(traj)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to reorder trajectory joints");
    return;
  }

  ActiveTrajectory at;
  at.traj = std::move(traj);
  at.start_time = get_node()->now();
  at.active = true;

  active_traj_buf_.writeFromNonRT(at);
  RCLCPP_DEBUG(get_node()->get_logger(),
      "Received trajectory with %zu points", msg->points.size());
}

bool MPCPositionController::reorderTrajectory(
    trajectory_msgs::msg::JointTrajectory& traj) const
{
  if (traj.joint_names.size() != num_joints_) {
    // Allow partial trajectories? For now require exact match
    if (traj.joint_names.size() < num_joints_) {
      RCLCPP_WARN(get_node()->get_logger(),
          "Trajectory has %zu joints, expected %zu", traj.joint_names.size(), num_joints_);
    }
  }

  // Build mapping: local_index -> traj_index
  std::vector<int> mapping(num_joints_, -1);
  for (size_t i = 0; i < num_joints_; ++i) {
    for (size_t j = 0; j < traj.joint_names.size(); ++j) {
      if (traj.joint_names[j] == joint_names_[i]) {
        mapping[i] = static_cast<int>(j);
        break;
      }
    }
    if (mapping[i] < 0) {
      RCLCPP_ERROR(get_node()->get_logger(),
          "Joint %s not found in trajectory", joint_names_[i].c_str());
      return false;
    }
  }

  // Reorder each point
  for (auto& pt : traj.points) {
    auto old_pos = pt.positions;
    auto old_vel = pt.velocities;
    auto old_acc = pt.accelerations;

    pt.positions.resize(num_joints_);
    pt.velocities.resize(num_joints_, 0.0);
    pt.accelerations.resize(num_joints_, 0.0);

    for (size_t i = 0; i < num_joints_; ++i) {
      int j = mapping[i];
      pt.positions[i] = (j < static_cast<int>(old_pos.size())) ? old_pos[j] : 0.0;
      pt.velocities[i] = (j < static_cast<int>(old_vel.size())) ? old_vel[j] : 0.0;
      pt.accelerations[i] = (j < static_cast<int>(old_acc.size())) ? old_acc[j] : 0.0;
    }
  }

  traj.joint_names = joint_names_;
  return true;
}

// ============================================================================
// Reference building
// ============================================================================

MPCReference MPCPositionController::buildReference(const rclcpp::Time& current_time)
{
  const int N = mpc_config_.horizon;
  const int n = static_cast<int>(num_joints_);
  const double dt = mpc_config_.dt;

  MPCReference ref;
  ref.q_ref.resize(N);
  ref.q_dot_ref.resize(N);

  if (is_holding_) {
    // Hold position: all reference points are the same
    Eigen::VectorXd zero_vel = Eigen::VectorXd::Zero(n);
    for (int k = 0; k < N; ++k) {
      ref.q_ref[k] = hold_position_;
      ref.q_dot_ref[k] = zero_vel;
    }
    return ref;
  }

  // Active trajectory: interpolate at each prediction step
  const auto& active_traj = *active_traj_buf_.readFromRT();
  if (!active_traj.active || active_traj.traj.points.empty()) {
    // Fallback to hold
    Eigen::VectorXd zero_vel = Eigen::VectorXd::Zero(n);
    for (int k = 0; k < N; ++k) {
      ref.q_ref[k] = hold_position_;
      ref.q_dot_ref[k] = zero_vel;
    }
    return ref;
  }

  const auto& points = active_traj.traj.points;
  double elapsed = (current_time - active_traj.start_time).seconds();
  double traj_duration = points.back().time_from_start.sec +
                         points.back().time_from_start.nanosec * 1e-9;

  for (int k = 0; k < N; ++k) {
    double t_query = elapsed + (k + 1) * dt;
    Eigen::VectorXd q_ref(n), v_ref(n);

    if (t_query >= traj_duration) {
      // Past end: hold final point
      const auto& last = points.back();
      for (int j = 0; j < n; ++j) {
        q_ref(j) = (j < static_cast<int>(last.positions.size())) ? last.positions[j] : 0.0;
        v_ref(j) = 0.0;  // zero velocity at end
      }
    } else if (t_query <= 0.0) {
      // Before start: use first point
      const auto& first = points.front();
      for (int j = 0; j < n; ++j) {
        q_ref(j) = (j < static_cast<int>(first.positions.size())) ? first.positions[j] : 0.0;
        v_ref(j) = (j < static_cast<int>(first.velocities.size())) ? first.velocities[j] : 0.0;
      }
    } else {
      // Find the two surrounding trajectory points
      size_t seg = 0;
      for (size_t s = 0; s < points.size() - 1; ++s) {
        double t_next = points[s + 1].time_from_start.sec +
                        points[s + 1].time_from_start.nanosec * 1e-9;
        if (t_query <= t_next) {
          seg = s;
          break;
        }
        seg = s;
      }

      double t0 = points[seg].time_from_start.sec +
                   points[seg].time_from_start.nanosec * 1e-9;
      double t1 = points[seg + 1].time_from_start.sec +
                   points[seg + 1].time_from_start.nanosec * 1e-9;

      interpolatePoint(points[seg], points[seg + 1], t0, t1, t_query, q_ref, v_ref);
    }

    ref.q_ref[k] = q_ref;
    ref.q_dot_ref[k] = v_ref;
  }

  return ref;
}

void MPCPositionController::interpolatePoint(
    const trajectory_msgs::msg::JointTrajectoryPoint& p0,
    const trajectory_msgs::msg::JointTrajectoryPoint& p1,
    double t0, double t1, double t_query,
    Eigen::VectorXd& q_out, Eigen::VectorXd& v_out) const
{
  const int n = static_cast<int>(num_joints_);
  double alpha = (t1 > t0) ? (t_query - t0) / (t1 - t0) : 0.0;
  alpha = std::clamp(alpha, 0.0, 1.0);

  q_out.resize(n);
  v_out.resize(n);

  for (int j = 0; j < n; ++j) {
    double pos0 = (j < static_cast<int>(p0.positions.size())) ? p0.positions[j] : 0.0;
    double pos1 = (j < static_cast<int>(p1.positions.size())) ? p1.positions[j] : 0.0;
    q_out(j) = pos0 + alpha * (pos1 - pos0);

    double vel0 = (j < static_cast<int>(p0.velocities.size())) ? p0.velocities[j] : 0.0;
    double vel1 = (j < static_cast<int>(p1.velocities.size())) ? p1.velocities[j] : 0.0;
    v_out(j) = vel0 + alpha * (vel1 - vel0);
  }
}

// ============================================================================
// FollowJointTrajectory action server
// ============================================================================

rclcpp_action::GoalResponse MPCPositionController::handleGoal(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const FollowJTAction::Goal> goal)
{
  if (goal->trajectory.points.empty()) {
    RCLCPP_WARN(get_node()->get_logger(), "Rejecting empty trajectory goal");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(get_node()->get_logger(),
      "Accepting trajectory goal with %zu points", goal->trajectory.points.size());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MPCPositionController::handleCancel(
    const std::shared_ptr<GoalHandleFJT> /*goal_handle*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "Cancelling trajectory goal");
  // Switch to holding current position
  hold_position_ = current_q_;
  is_holding_ = true;
  goal_active_.store(false);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MPCPositionController::handleAccepted(
    const std::shared_ptr<GoalHandleFJT> goal_handle)
{
  // Cancel previous goal if any
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_handle_) {
      auto result = std::make_shared<FollowJTAction::Result>();
      result->error_code = FollowJTAction::Result::INVALID_GOAL;
      result->error_string = "Preempted by new goal";
      active_goal_handle_->abort(result);
    }
    active_goal_handle_ = goal_handle;
  }

  // Feed trajectory into the same pipeline as topic subscription
  auto traj = goal_handle->get_goal()->trajectory;
  if (!reorderTrajectory(traj)) {
    auto result = std::make_shared<FollowJTAction::Result>();
    result->error_code = FollowJTAction::Result::INVALID_JOINTS;
    result->error_string = "Joint names mismatch";
    goal_handle->abort(result);
    std::lock_guard<std::mutex> lock(goal_mutex_);
    active_goal_handle_.reset();
    return;
  }

  ActiveTrajectory at;
  at.traj = std::move(traj);
  at.start_time = get_node()->now();
  at.active = true;
  active_traj_buf_.writeFromNonRT(at);

  goal_active_.store(true);
  RCLCPP_INFO(get_node()->get_logger(), "Executing trajectory goal");
}

}  // namespace mpc_position_controller

// ============================================================================
// Plugin registration
// ============================================================================
PLUGINLIB_EXPORT_CLASS(
    mpc_position_controller::MPCPositionController,
    controller_interface::ControllerInterface)
