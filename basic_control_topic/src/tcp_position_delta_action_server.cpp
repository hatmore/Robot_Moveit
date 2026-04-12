#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "planning_sdk_msgs/action/tcp_position_delta.hpp"
#include "planning_sdk_msgs/msg/tcp_pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <unordered_map>
#include <mutex>
#include <cmath>
#include <chrono>
#include "basic_control_topic/joint_state_monitor.hpp"
#include "planning_sdk_msgs/msg/joint_limits_array.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"

using namespace std::placeholders;
using TcpPositionDelta = planning_sdk_msgs::action::TcpPositionDelta;
using GoalHandleTcpPositionDelta = rclcpp_action::ServerGoalHandle<TcpPositionDelta>;
using TcpPoseMsg = planning_sdk_msgs::msg::TcpPose;

using namespace std::chrono_literals;

struct GoalUUIDHash {
  size_t operator()(const rclcpp_action::GoalUUID& uuid) const {
    return std::hash<std::string>()(std::string(uuid.begin(), uuid.end()));
  }
};

class TcpPositionDeltaActionServer : public rclcpp::Node {
public:
  TcpPositionDeltaActionServer() : Node("tcp_position_delta_action_server"), js_monitor_(this) {
    this->declare_parameter<std::string>("right_end_effector_link", "right_gripper_base_link");
    right_end_effector_link_ = this->get_parameter("right_end_effector_link").as_string();
    
    left_action_server_ = rclcpp_action::create_server<TcpPositionDelta>(
      this,
      "/algorithm/grasp_planning/move/left_arm/tcp_position_delta",
      std::bind(&TcpPositionDeltaActionServer::handle_goal, this, _1, _2, "left"),
      std::bind(&TcpPositionDeltaActionServer::handle_cancel, this, _1),
      std::bind(&TcpPositionDeltaActionServer::handle_accepted, this, _1)
    );

    right_action_server_ = rclcpp_action::create_server<TcpPositionDelta>(
      this,
      "/algorithm/grasp_planning/move/right_arm/tcp_position_delta",
      std::bind(&TcpPositionDeltaActionServer::handle_goal, this, _1, _2, "right"),
      std::bind(&TcpPositionDeltaActionServer::handle_cancel, this, _1),
      std::bind(&TcpPositionDeltaActionServer::handle_accepted, this, _1)
    );

    left_tcp_pose_sub_ = this->create_subscription<TcpPoseMsg>(
      "/algorithm/grasp_planning/left_arm/tcp_pose",
      10,
      std::bind(&TcpPositionDeltaActionServer::left_tcp_pose_callback, this, _1)
    );

    right_tcp_pose_sub_ = this->create_subscription<TcpPoseMsg>(
      "/algorithm/grasp_planning/right_arm/tcp_pose",
      10,
      std::bind(&TcpPositionDeltaActionServer::right_tcp_pose_callback, this, _1)
    );

    left_pose_received_ = false;
    right_pose_received_ = false;

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

    RCLCPP_INFO(this->get_logger(), "末端位置偏移控制Action服务器已启动（订阅话题获取位姿）");
    RCLCPP_INFO(this->get_logger(), "订阅左臂位姿话题：%s", "/algorithm/grasp_planning/left_arm/tcp_pose");
    RCLCPP_INFO(this->get_logger(), "订阅右臂位姿话题：%s", "/algorithm/grasp_planning/right_arm/tcp_pose");
    RCLCPP_INFO(this->get_logger(), "右臂末端Link：%s", right_end_effector_link_.c_str());
  }

private:
  rclcpp_action::Server<TcpPositionDelta>::SharedPtr left_action_server_;
  rclcpp_action::Server<TcpPositionDelta>::SharedPtr right_action_server_;
  std::unordered_map<rclcpp_action::GoalUUID, std::string, GoalUUIDHash> goal_arm_map_;
  std::mutex map_mutex_;
  std::mutex execute_mutex_;           // 保证 execute 串行执行，防止并发导致位姿回退

  rclcpp::Subscription<TcpPoseMsg>::SharedPtr left_tcp_pose_sub_;
  rclcpp::Subscription<TcpPoseMsg>::SharedPtr right_tcp_pose_sub_;
  TcpPoseMsg latest_left_tcp_pose_;
  TcpPoseMsg latest_right_tcp_pose_;
  std::mutex left_pose_mutex_;
  std::mutex right_pose_mutex_;
  bool left_pose_received_;
  bool right_pose_received_;
  std::string right_end_effector_link_;
  std::string left_end_effector_link_ = "left_gripper_base_link";
  JointStateMonitor js_monitor_;

  struct PosLimit { double min_pos; double max_pos; };
  std::unordered_map<std::string, PosLimit> joint_pos_limits_;
  std::mutex limits_mutex_;
  rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;

  void left_tcp_pose_callback(const TcpPoseMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(left_pose_mutex_);
    latest_left_tcp_pose_ = *msg;
    left_pose_received_ = (msg->error_code == 0);
    if (left_pose_received_) {
      RCLCPP_DEBUG(this->get_logger(), "[左臂] 收到有效位姿：x=%.3f, y=%.3f, z=%.3f",
                   msg->current_pose.position.x,
                   msg->current_pose.position.y,
                   msg->current_pose.position.z);
    } else {
      RCLCPP_WARN(this->get_logger(), "[左臂] 收到无效位姿，错误码：%d", msg->error_code);
    }
  }

  void right_tcp_pose_callback(const TcpPoseMsg::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(right_pose_mutex_);
    latest_right_tcp_pose_ = *msg;
    right_pose_received_ = (msg->error_code == 0);
    if (right_pose_received_) {
      RCLCPP_DEBUG(this->get_logger(), "[右臂] 收到有效位姿：x=%.3f, y=%.3f, z=%.3f",
                   msg->current_pose.position.x,
                   msg->current_pose.position.y,
                   msg->current_pose.position.z);
    } else {
      RCLCPP_WARN(this->get_logger(), "[右臂] 收到无效位姿，错误码：%d", msg->error_code);
    }
  }

  bool get_latest_tcp_pose(const std::string& arm_type, geometry_msgs::msg::PoseStamped& current_pose) {
    int retry_count = 0;
    const int max_retry = 6;
    const std::chrono::milliseconds retry_delay = 500ms;

    while (retry_count < max_retry && rclcpp::ok()) {
      if (arm_type == "left") {
        std::lock_guard<std::mutex> lock(left_pose_mutex_);
        if (left_pose_received_) {
          current_pose.header.frame_id = "base_link";
          current_pose.header.stamp = latest_left_tcp_pose_.timestamp;
          current_pose.pose = latest_left_tcp_pose_.current_pose;
          return true;
        }
      } else if (arm_type == "right") {
        std::lock_guard<std::mutex> lock(right_pose_mutex_);
        if (right_pose_received_) {
          current_pose.header.frame_id = "base_link";
          current_pose.header.stamp = latest_right_tcp_pose_.timestamp;
          current_pose.pose = latest_right_tcp_pose_.current_pose;
          return true;
        }
      }

      retry_count++;
      RCLCPP_WARN(this->get_logger(), "[%s臂] 未收到有效位姿（第%d次重试），等待订阅消息...",
                   arm_type.c_str(), retry_count);
      std::this_thread::sleep_for(retry_delay);
    }

    RCLCPP_ERROR(this->get_logger(), "[%s臂] 超时未收到有效位姿（3秒）", arm_type.c_str());
    return false;
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const TcpPositionDelta::Goal> goal,
    const std::string& arm_type) {
    double quat_norm = std::sqrt(
      goal->pose_delta.pose.orientation.x * goal->pose_delta.pose.orientation.x +
      goal->pose_delta.pose.orientation.y * goal->pose_delta.pose.orientation.y +
      goal->pose_delta.pose.orientation.z * goal->pose_delta.pose.orientation.z +
      goal->pose_delta.pose.orientation.w * goal->pose_delta.pose.orientation.w
    );
    if (std::abs(quat_norm - 1.0) > 0.01) {
      RCLCPP_ERROR(this->get_logger(), "[%s臂] 偏移姿态四元数未归一化（模长=%.3f），拒绝请求",
                   arm_type.c_str(), quat_norm);
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (goal->pose_delta.header.frame_id != "base_link") {
      RCLCPP_WARN(this->get_logger(), "[%s臂] 偏移参考系为'%s'，强制使用base_link",
                   arm_type.c_str(), goal->pose_delta.header.frame_id.c_str());
      // return rclcpp_action::GoalResponse::REJECT;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    goal_arm_map_[uuid] = arm_type;
    RCLCPP_INFO(this->get_logger(), "[%s臂] 收到末端偏移请求，旋转样式: %d",
                 arm_type.c_str(), goal->rotation_style);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "收到末端偏移取消请求，清理目标记录");
    std::lock_guard<std::mutex> lock(map_mutex_);
    goal_arm_map_.erase(goal_handle->get_goal_id());
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle) {
    std::thread{std::bind(&TcpPositionDeltaActionServer::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle) {
    // 串行化执行，防止并发请求导致位姿回退
    std::lock_guard<std::mutex> exec_lock(execute_mutex_);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TcpPositionDelta::Feedback>();
    auto result = std::make_shared<TcpPositionDelta::Result>();
    result->error_code = 0;

    rclcpp_action::GoalUUID goal_id = goal_handle->get_goal_id();
    std::string arm_type;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      auto it = goal_arm_map_.find(goal_id);
      if (it == goal_arm_map_.end()) {
        RCLCPP_ERROR(this->get_logger(), "未找到目标ID对应的手臂类型，执行失败");
        result->error_code = 4;
        goal_handle->abort(result);
        return;
      }
      arm_type = it->second;
    }

    std::string move_group_name = (arm_type == "left") ? "left_arm_group" : "right_arm_group";
    std::string end_effector_link = (arm_type == "left") ? left_end_effector_link_ : right_end_effector_link_;

    geometry_msgs::msg::PoseStamped current_pose;
    bool get_pose_success = get_latest_tcp_pose(arm_type, current_pose);

    if (!get_pose_success) {
      RCLCPP_ERROR(this->get_logger(), "[%s臂] 无法获取当前位姿，执行失败", arm_type.c_str());
      result->error_code = 4;
      goal_handle->abort(result);
      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_.erase(goal_id);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "[%s臂] 当前位姿：x=%.3f, y=%.3f, z=%.3f",
                 arm_type.c_str(),
                 current_pose.pose.position.x,
                 current_pose.pose.position.y,
                 current_pose.pose.position.z);

    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    target_pose.pose.position.x += goal->pose_delta.pose.position.x;
    target_pose.pose.position.y += goal->pose_delta.pose.position.y;
    target_pose.pose.position.z += goal->pose_delta.pose.position.z;

    tf2::Quaternion current_quat;
    tf2::Quaternion delta_quat;
    tf2::Quaternion target_quat;

    tf2::fromMsg(current_pose.pose.orientation, current_quat);
    tf2::fromMsg(goal->pose_delta.pose.orientation, delta_quat);

    target_quat = delta_quat * current_quat;
    target_quat.normalize();

    target_pose.pose.orientation = tf2::toMsg(target_quat);

    RCLCPP_INFO(this->get_logger(), "[%s臂] 偏移后目标位姿：x=%.3f, y=%.3f, z=%.3f",
                 arm_type.c_str(),
                 target_pose.pose.position.x,
                 target_pose.pose.position.y,
                 target_pose.pose.position.z);

    // -------------------------- 核心修改：无偏移请求直接返回成功，不执行规划 --------------------------
    double pos_offset_mag = std::sqrt(
      std::pow(goal->pose_delta.pose.position.x, 2) +
      std::pow(goal->pose_delta.pose.position.y, 2) +
      std::pow(goal->pose_delta.pose.position.z, 2)
    );
    double orient_offset_dot = std::abs(
      tf2::Quaternion(goal->pose_delta.pose.orientation.x, goal->pose_delta.pose.orientation.y,
                      goal->pose_delta.pose.orientation.z, goal->pose_delta.pose.orientation.w)
      .dot(tf2::Quaternion(0.0, 0.0, 0.0, 1.0))
    );

    // 只要是无偏移请求（位置偏移<1mm + 姿态偏移<0.5°），直接返回成功
    if (pos_offset_mag < 1e-3 && orient_offset_dot > 0.99999) {
      RCLCPP_INFO(this->get_logger(), "[%s臂] 触发无偏移请求，强制不移动，直接返回成功", arm_type.c_str());
      
      feedback->timestamp = this->get_clock()->now();
      feedback->process_status = 100;
      goal_handle->publish_feedback(feedback);
      result->timestamp = this->get_clock()->now();
      result->planning_time.sec = 0;
      result->planning_time.nanosec = 0;
      goal_handle->succeed(result);

      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_.erase(goal_id);
      return;
    }
    // ------------------------------------------------------------------------------------------

    rclcpp::Node::SharedPtr moveit_node = rclcpp::Node::make_shared("moveit_tcp_delta_node");
    rclcpp::executors::SingleThreadedExecutor moveit_executor;
    moveit_executor.add_node(moveit_node);
    std::thread moveit_thread([&moveit_executor]() { moveit_executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(moveit_node, move_group_name);
    move_group.setEndEffectorLink(end_effector_link);
    move_group.setPlanningTime(15.0);
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.01);
    double vel_scale = (goal->velocity == 0) ? 0.5 : goal->velocity / 100.0 * 0.5;
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(0.5);
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("PTP");

    // Build path constraints: soft limits + optional orientation
    {
      std::vector<std::string> group_joints = move_group.getJoints();
      moveit_msgs::msg::Constraints path_constraints;
      {
        std::lock_guard<std::mutex> lock(limits_mutex_);
        for (const auto& jname : group_joints) {
          auto lit = joint_pos_limits_.find(jname);
          if (lit != joint_pos_limits_.end()) {
            double center = (lit->second.min_pos + lit->second.max_pos) / 2.0;
            moveit_msgs::msg::JointConstraint jc;
            jc.joint_name = jname;
            jc.position = center;
            jc.tolerance_above = lit->second.max_pos - center;
            jc.tolerance_below = center - lit->second.min_pos;
            jc.weight = 1.0;
            path_constraints.joint_constraints.push_back(jc);
          }
        }
      }
      if (goal->rotation_style == 1) {
        moveit_msgs::msg::OrientationConstraint ocm;
        ocm.link_name = end_effector_link;
        ocm.header.frame_id = "base_link";
        ocm.orientation = target_pose.pose.orientation;
        ocm.absolute_x_axis_tolerance = 0.01;
        ocm.absolute_y_axis_tolerance = 0.01;
        ocm.absolute_z_axis_tolerance = 0.01;
        ocm.weight = 1.0;
        path_constraints.orientation_constraints.push_back(ocm);
      }
      move_group.setPathConstraints(path_constraints);
      RCLCPP_INFO(this->get_logger(), "[%s臂] Set %zu joint + %zu orientation path constraints",
                  arm_type.c_str(), path_constraints.joint_constraints.size(),
                  path_constraints.orientation_constraints.size());
    }

    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success = (move_group.plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_success) {
      RCLCPP_ERROR(this->get_logger(), "[%s臂] 轨迹规划失败，可能位姿不可达或存在碰撞",
                   arm_type.c_str());
      result->error_code = 2;
      goal_handle->abort(result);
      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_.erase(goal_id);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[%s臂] 规划成功，耗时: %.2f秒",
                 arm_type.c_str(), motion_plan.planning_time_);

    std::vector<std::string> joint_names = move_group.getJoints();
    bool execute_success = js_monitor_.executeWithTimeout(move_group, motion_plan, 3.0);

    if (!execute_success) {
      RCLCPP_ERROR(this->get_logger(), "[%s臂] 轨迹执行失败", arm_type.c_str());
      result->error_code = 3;
      goal_handle->abort(result);
      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_.erase(goal_id);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    // 验证是否真的到达目标
    auto final_state = move_group.getCurrentState();
    std::vector<double> final_positions;
    final_state->copyJointGroupPositions(move_group_name, final_positions);

    if (!js_monitor_.verifyExecution(joint_names, final_positions)) {
      RCLCPP_ERROR(this->get_logger(), "[%s臂] 执行验证失败，SDK可能已挂", arm_type.c_str());
      result->error_code = 3;
      goal_handle->abort(result);
      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_.erase(goal_id);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    feedback->timestamp = this->get_clock()->now();
    feedback->process_status = 100;
    goal_handle->publish_feedback(feedback);

    result->timestamp = this->get_clock()->now();
    result->planning_time.sec = static_cast<int>(motion_plan.planning_time_);
    result->planning_time.nanosec = static_cast<int>(
      (motion_plan.planning_time_ - result->planning_time.sec) * 1e9
    );
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "[%s臂] 末端偏移控制执行成功！", arm_type.c_str());

    std::lock_guard<std::mutex> lock(map_mutex_);
    goal_arm_map_.erase(goal_id);
    moveit_executor.cancel();
    moveit_thread.join();
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto server_node = std::make_shared<TcpPositionDeltaActionServer>();
  rclcpp::spin(server_node);
  rclcpp::shutdown();
  return 0;
}
