#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "planning_sdk_msgs/action/joint_position.hpp"
#include "planning_sdk_msgs/msg/joint_limits_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include <functional>
#include <mutex>
#include <unordered_map>
#include <algorithm>
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/display_robot_state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "basic_control_topic/joint_state_monitor.hpp"

// 时间参数化速度/加速度缩放（1.0 = 使用 joint_limits.yaml 中的最大值）
static constexpr double TOTG_VELOCITY_SCALING     = 0.05;
static constexpr double TOTG_ACCELERATION_SCALING = 0.2;

using namespace std::placeholders;
using JointPosition = planning_sdk_msgs::action::JointPosition;
using GoalHandleJointPosition = rclcpp_action::ServerGoalHandle<JointPosition>;

class JointPositionActionServer : public rclcpp::Node {
public:
  JointPositionActionServer() : Node("joint_position_server"), js_monitor_(this) {
    // 订阅动态关节限位
    auto limits_qos = rclcpp::QoS(1).transient_local();
    joint_limits_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointLimitsArray>(
      "/algorithm/joint_limits/current", limits_qos,
      [this](const planning_sdk_msgs::msg::JointLimitsArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(limits_mutex_);
        joint_pos_limits_.clear();
        for (const auto& jl : msg->limits) {
          joint_pos_limits_[jl.joint_name] = {jl.lower_limit, jl.upper_limit};
        }
        RCLCPP_INFO(this->get_logger(), "Updated joint position limits (%zu joints)", msg->limits.size());
      });

    action_server_ = rclcpp_action::create_server<JointPosition>(
      this,
      "/algorithm/grasp_planning/move/joint_position",
      std::bind(&JointPositionActionServer::handle_goal, this, _1, _2),
      std::bind(&JointPositionActionServer::handle_cancel, this, _1),
      std::bind(&JointPositionActionServer::handle_accepted, this, _1)
    );
    RCLCPP_INFO(this->get_logger(), "关节移动 Action 服务器已启动");
  }

private:
  rclcpp_action::Server<JointPosition>::SharedPtr action_server_;
  rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;
  struct PosLimit { double min_pos; double max_pos; };
  std::unordered_map<std::string, PosLimit> joint_pos_limits_;
  std::mutex limits_mutex_;
  JointStateMonitor js_monitor_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const JointPosition::Goal> goal) {
    if (goal->target_state.name.size() != 7) {
      RCLCPP_ERROR(this->get_logger(), "目标关节数量错误！需7个关节（左右臂），当前为 %zu 个",
                   goal->target_state.name.size());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // Check target positions against soft limits before accepting
    {
      std::lock_guard<std::mutex> lock(limits_mutex_);
      for (size_t i = 0; i < goal->target_state.name.size() && i < goal->target_state.position.size(); ++i) {
        auto lit = joint_pos_limits_.find(goal->target_state.name[i]);
        if (lit != joint_pos_limits_.end()) {
          double pos = goal->target_state.position[i];
          if (pos < lit->second.min_pos || pos > lit->second.max_pos) {
            RCLCPP_ERROR(this->get_logger(),
              "Goal REJECTED: [%s] target=%.3f exceeds soft limits [%.3f, %.3f]",
              goal->target_state.name[i].c_str(), pos, lit->second.min_pos, lit->second.max_pos);
            return rclcpp_action::GoalResponse::REJECT;
          }
        }
      }
    }

    std::string joint_names;
    for (const auto& name : goal->target_state.name) {
      joint_names += name + ", ";
    }
    RCLCPP_INFO(this->get_logger(), "收到关节移动请求，关节列表：%s", joint_names.c_str());
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleJointPosition> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "收到关节移动取消请求");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleJointPosition> goal_handle) {
    std::thread{std::bind(&JointPositionActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleJointPosition> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<JointPosition::Feedback>();
    auto result = std::make_shared<JointPosition::Result>();

    rclcpp::Node::SharedPtr moveit_node = rclcpp::Node::make_shared("moveit_joint_control_node");
    rclcpp::executors::SingleThreadedExecutor moveit_executor;
    moveit_executor.add_node(moveit_node);
    std::thread moveit_thread([&moveit_executor]() { moveit_executor.spin(); });

    std::string move_group_name;
    if (goal->target_state.name[0].find("left_") != std::string::npos) {
      move_group_name = "left_arm_group";
    } else if (goal->target_state.name[0].find("right_") != std::string::npos) {
      move_group_name = "right_arm_group";
    } else {
      RCLCPP_ERROR(this->get_logger(), "关节名称无 'left_' 或 'right_' 前缀，无法识别关节组");
      result->error_code = 4;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    moveit::planning_interface::MoveGroupInterface move_group(moveit_node, move_group_name);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(30.0);
    move_group.setGoalJointTolerance(0.01);
    move_group.setGoalPositionTolerance(0.005);

    // Use original target positions without clamping
    std::vector<double> target_positions(goal->target_state.position.begin(), goal->target_state.position.end());

    // Add joint constraints based on dynamic limits
    moveit_msgs::msg::Constraints joint_constraints;
    {
      std::lock_guard<std::mutex> lock(limits_mutex_);
      for (size_t i = 0; i < goal->target_state.name.size() && i < target_positions.size(); ++i) {
        auto lit = joint_pos_limits_.find(goal->target_state.name[i]);
        if (lit != joint_pos_limits_.end()) {
          double center = (lit->second.min_pos + lit->second.max_pos) / 2.0;
          moveit_msgs::msg::JointConstraint jc;
          jc.joint_name = goal->target_state.name[i];
          jc.position = center;
          jc.tolerance_above = lit->second.max_pos - center;
          jc.tolerance_below = center - lit->second.min_pos;
          jc.weight = 1.0;
          joint_constraints.joint_constraints.push_back(jc);

          RCLCPP_INFO(this->get_logger(), "Added constraint [%s] center=%.3f range=[%.3f, %.3f] tol=[%.3f, %.3f]",
                      goal->target_state.name[i].c_str(), center,
                      lit->second.min_pos, lit->second.max_pos,
                      jc.tolerance_below, jc.tolerance_above);
        }
      }
    }
    move_group.setPathConstraints(joint_constraints);
    RCLCPP_INFO(this->get_logger(), "Set %zu joint constraints for planning", joint_constraints.joint_constraints.size());

    move_group.setJointValueTarget(goal->target_state.name, target_positions);
    RCLCPP_INFO(this->get_logger(), "已设置 %s 目标关节角，开始规划路径", move_group_name.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success = false;

    // Try PTP first (Pilz planner)
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("PTP");
    plan_success = (move_group.plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // If PTP fails, try RRTConnect (OMPL)
    if (!plan_success) {
      RCLCPP_WARN(this->get_logger(), "PTP planning failed, trying RRTConnect with OMPL...");
      move_group.setPlanningPipelineId("ompl");
      move_group.setPlannerId("RRTConnect");
      plan_success = (move_group.plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    if (!plan_success) {
      RCLCPP_ERROR(this->get_logger(), "%s 运动规划失败！请检查关节角度是否超限或存在碰撞",
                   move_group_name.c_str());
      result->timestamp = this->get_clock()->now();
      result->error_code = 2;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "%s 规划成功！规划耗时：%.2f 秒",
                 move_group_name.c_str(), motion_plan.planning_time_);

    // Verify trajectory respects joint constraints
    {
      std::lock_guard<std::mutex> lock(limits_mutex_);
      bool constraint_violated = false;
      for (const auto& point : motion_plan.trajectory_.joint_trajectory.points) {
        for (size_t i = 0; i < motion_plan.trajectory_.joint_trajectory.joint_names.size() && i < point.positions.size(); ++i) {
          const auto& joint_name = motion_plan.trajectory_.joint_trajectory.joint_names[i];
          auto lit = joint_pos_limits_.find(joint_name);
          if (lit != joint_pos_limits_.end()) {
            double pos = point.positions[i];
            if (pos < lit->second.min_pos || pos > lit->second.max_pos) {
              RCLCPP_ERROR(this->get_logger(), "Constraint VIOLATED [%s] pos=%.3f not in [%.3f, %.3f]",
                          joint_name.c_str(), pos, lit->second.min_pos, lit->second.max_pos);
              constraint_violated = true;
            }
          }
        }
      }
      if (!constraint_violated) {
        RCLCPP_INFO(this->get_logger(), "Constraint verification PASSED: all trajectory points within limits");
      }
    }

    // 手动进行时间参数化（MoveIt2 的 response adapter 可能未生效）
    robot_trajectory::RobotTrajectory robot_traj(move_group.getRobotModel(), move_group_name);
    robot_traj.setRobotTrajectoryMsg(*move_group.getCurrentState(), motion_plan.trajectory_);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    double vel_scale = (goal->velocity == 0) ? TOTG_VELOCITY_SCALING
                                          : goal->velocity / 100.0 * 0.5;
    bool time_param_success = totg.computeTimeStamps(robot_traj, vel_scale, vel_scale);

    if (time_param_success) {
        robot_traj.getRobotTrajectoryMsg(motion_plan.trajectory_);
        RCLCPP_INFO(this->get_logger(), "时间参数化成功，轨迹点数: %zu",
                    motion_plan.trajectory_.joint_trajectory.points.size());
    } else {
        RCLCPP_ERROR(this->get_logger(), "时间参数化失败！");
        result->timestamp = this->get_clock()->now();
        result->error_code = 5;
        goal_handle->abort(result);
        moveit_executor.cancel();
        moveit_thread.join();
        return;
    }

    // 尝试执行运动
    bool execute_success = js_monitor_.executeWithTimeout(move_group, motion_plan, 3.0);

    if (!execute_success) {
      RCLCPP_ERROR(this->get_logger(), "%s 运动执行失败", move_group_name.c_str());
      result->timestamp = this->get_clock()->now();
      result->error_code = 3;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    // 验证是否真的到达目标
    if (!js_monitor_.verifyExecution(goal->target_state.name, target_positions)) {
      RCLCPP_ERROR(this->get_logger(), "%s 执行验证失败，SDK可能已挂", move_group_name.c_str());
      result->timestamp = this->get_clock()->now();
      result->error_code = 3;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    feedback->timestamp = this->get_clock()->now();
    feedback->process_status = 100;
    goal_handle->publish_feedback(feedback);

    result->timestamp = this->get_clock()->now();
    result->planning_time.sec = static_cast<int>(motion_plan.planning_time_);
    result->planning_time.nanosec = static_cast<int>((motion_plan.planning_time_ - result->planning_time.sec) * 1e9);
    result->error_code = 0;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "%s 已成功移动到目标位置！", move_group_name.c_str());

    moveit_executor.cancel();
    moveit_thread.join();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointPositionActionServer>());
  rclcpp::shutdown();
  return 0;
}
