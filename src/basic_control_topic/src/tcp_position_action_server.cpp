/**
 * ================================================================================
 * TCP 末端位置控制服务器 (TCP Position Action Server)
 * ================================================================================
 *
 * 功能: 接收来自客户端的末端（Tool Center Point, TCP）绝对位置请求，
 *      通过 MoveIt 进行逆运动学计算和轨迹规划，执行末端位置控制
 *
 * 核心特点:
 *   - 支持双臂独立控制（左右臂分别的 Action Topic）
 *   - 四元数验证（确保输入的旋转表示有效）
 *   - 线程安全的目标映射（Mutex 保护）
 *   - 离线规划设计（规划和执行都在 execute 中）
 *
 * 调用流程:
 *   1. Client 发送 Goal (末端位姿: 位置 + 旋转)
 *   2. handle_goal() -> 四元数验证 + UUID 映射
 *   3. handle_accepted() -> 创建独立线程
 *   4. execute() -> 规划末端路径 -> 执行运动
 *
 * ================================================================================
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "planning_sdk_msgs/action/tcp_position.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include <unordered_map>
#include <mutex>
#include "basic_control_topic/joint_state_monitor.hpp"
#include "planning_sdk_msgs/msg/joint_limits_array.hpp"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"

using namespace std::placeholders;
using TcpPosition = planning_sdk_msgs::action::TcpPosition;
using GoalHandleTcpPosition = rclcpp_action::ServerGoalHandle<TcpPosition>;
using GoalUUID = rclcpp_action::GoalUUID;

/**
 * 自定义哈希函数: 使 GoalUUID 可以用作 unordered_map 的键
 *
 * GoalUUID 是一个 std::array<uint8_t, 16>，需要自定义哈希函数
 * 才能被 unordered_map 使用
 *
 * 输入: uuid - 16 字节的 UUID
 * 输出: 返回 UUID 对应的哈希值
 */
struct GoalUUIDHash {
  size_t operator()(const GoalUUID& uuid) const {
    // 将 UUID 字节序列转换为字符串，然后哈希
    return std::hash<std::string>()(std::string(uuid.begin(), uuid.end()));
  }
};

class TcpPositionActionServer : public rclcpp::Node {
public:

  TcpPositionActionServer() : Node("tcp_position_action_server"), js_monitor_(this) {
    left_action_server_ = rclcpp_action::create_server<TcpPosition>(
      this,
      "/algorithm/grasp_planning/move/left_arm/tcp_position",
      std::bind(&TcpPositionActionServer::handle_goal, this, _1, _2, "left"),
      std::bind(&TcpPositionActionServer::handle_cancel, this, _1),
      std::bind(&TcpPositionActionServer::handle_accepted, this, _1)
    );

    // 创建右臂 Action 服务器
    // Topic: /algorithm/grasp_planning/move/right_arm/tcp_position
    right_action_server_ = rclcpp_action::create_server<TcpPosition>(
      this,
      "/algorithm/grasp_planning/move/right_arm/tcp_position",
      std::bind(&TcpPositionActionServer::handle_goal, this, _1, _2, "right"),
      std::bind(&TcpPositionActionServer::handle_cancel, this, _1),
      std::bind(&TcpPositionActionServer::handle_accepted, this, _1)
    );

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

    RCLCPP_INFO(this->get_logger(), "末端位置控制Action服务器已启动（左右臂话题）");
  }

private:
  rclcpp_action::Server<TcpPosition>::SharedPtr left_action_server_;
  rclcpp_action::Server<TcpPosition>::SharedPtr right_action_server_;

  // 线程安全的映射表: Goal UUID -> 手臂类型（"left" 或 "right"）
  // 用途: 在 execute 中根据 goal_id 查询该 Goal 属于哪个手臂
  std::unordered_map<GoalUUID, std::string, GoalUUIDHash> goal_arm_map_;
  std::mutex map_mutex_;  // 保护 goal_arm_map_ 的互斥锁
  JointStateMonitor js_monitor_;

  struct PosLimit { double min_pos; double max_pos; };
  std::unordered_map<std::string, PosLimit> joint_pos_limits_;
  std::mutex limits_mutex_;
  rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;


  rclcpp_action::GoalResponse handle_goal(
    const GoalUUID& uuid,
    std::shared_ptr<const TcpPosition::Goal> goal,
    const std::string& arm_type) {
    // 四元数有效性验证
    // 公式: |q|² = x² + y² + z² + w² = 1.0
    double quat_norm = goal->pose.pose.orientation.x*goal->pose.pose.orientation.x +
                       goal->pose.pose.orientation.y*goal->pose.pose.orientation.y +
                       goal->pose.pose.orientation.z*goal->pose.pose.orientation.z +
                       goal->pose.pose.orientation.w*goal->pose.pose.orientation.w;

    // 检查四元数模长是否接近 1.0（公差 0.01）
    if (std::abs(quat_norm - 1.0) > 0.01) {
      RCLCPP_ERROR(this->get_logger(), "目标姿态四元数未归一化（模长=%.3f），拒绝请求", quat_norm);
      return rclcpp_action::GoalResponse::REJECT;
    }

    // 保存 UUID 到手臂映射关系（线程安全）
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_[uuid] = arm_type;
    }

    RCLCPP_INFO(this->get_logger(), "收到%s臂末端移动请求，旋转样式: %d",
                 arm_type.c_str(), goal->rotation_style);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /**
   * 处理取消请求
   *
   * 调用时机: 客户端发送 cancel_goal 时
   * 输入: goal_handle - 要取消的 Goal 句柄
   * 输出: 从 goal_arm_map 中删除 UUID 映射
   */
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTcpPosition> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "收到末端移动取消请求");
    std::lock_guard<std::mutex> lock(map_mutex_);
    goal_arm_map_.erase(goal_handle->get_goal_id());
    return rclcpp_action::CancelResponse::ACCEPT;
  }


  void handle_accepted(const std::shared_ptr<GoalHandleTcpPosition> goal_handle) {
    std::thread{std::bind(&TcpPositionActionServer::execute, this, goal_handle)}.detach();
  }

 
  void execute(const std::shared_ptr<GoalHandleTcpPosition> goal_handle) {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TcpPosition::Feedback>();
    auto result = std::make_shared<TcpPosition::Result>();
    result->error_code = 0;

    // 查询该 Goal 对应的手臂类型
    GoalUUID goal_id = goal_handle->get_goal_id();
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

    // 根据手臂类型确定 MoveIt 关节组名
    std::string move_group_name = (arm_type == "left")
      ? "left_arm_group"
      : "right_arm_group";

    RCLCPP_INFO(this->get_logger(), "开始处理%s臂末端移动请求", arm_type.c_str());

    // 创建 MoveIt 独立 Executor（避免死锁）                                                                                                                                                                                      
    rclcpp::Node::SharedPtr moveit_node = rclcpp::Node::make_shared("moveit_tcp_node");
    rclcpp::executors::SingleThreadedExecutor moveit_executor;
    moveit_executor.add_node(moveit_node);
    std::thread moveit_thread([&moveit_executor]() { moveit_executor.spin(); });

    // 创建 MoveGroupInterface
    moveit::planning_interface::MoveGroupInterface move_group(moveit_node, move_group_name);
    move_group.setPlanningTime(10.0);
    move_group.setGoalPositionTolerance(0.005);      // 位置精度: 5mm
    move_group.setGoalOrientationTolerance(0.01);    // 旋转精度: ~0.57°

    // Add path constraints based on soft limits for this arm group
    {
      std::vector<std::string> group_joints = move_group.getJoints();
      moveit_msgs::msg::Constraints joint_constraints;
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
          joint_constraints.joint_constraints.push_back(jc);
        }
      }
      move_group.setPathConstraints(joint_constraints);
      RCLCPP_INFO(this->get_logger(), "%s臂: Set %zu joint path constraints",
                  arm_type.c_str(), joint_constraints.joint_constraints.size());
    }

    // 设置末端目标位姿
    move_group.setPoseTarget(goal->pose);
    RCLCPP_INFO(this->get_logger(), "目标位姿已设置，参考系: %s",
                 goal->pose.header.frame_id.c_str());

    // 规划末端运动 - 优先使用 Cartesian 直线规划
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success = false;

    // 尝试 Cartesian 直线规划（末端走直线）
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group.getCurrentPose().pose);  // 起点
    waypoints.push_back(goal->pose.pose);  // 终点（提取 .pose 部分）

    double fraction = move_group.computeCartesianPath(
      waypoints,
      0.01,  // eef_step: 末端步长 1cm
      0.0,   // jump_threshold: 关节跳跃阈值，0 表示不检查
      motion_plan.trajectory_
    );

    if (fraction >= 0.99) {  // 成功规划至少 99% 的路径
      plan_success = true;
      motion_plan.planning_time_ = 0.1;  // Cartesian 规划通常很快
      RCLCPP_INFO(this->get_logger(), "%s臂 Cartesian 直线规划成功，完成度: %.1f%%",
                   arm_type.c_str(), fraction * 100);
    } else if (fraction > 0) {
      RCLCPP_WARN(this->get_logger(), "%s臂 Cartesian 规划不完整（%.1f%%），尝试 OMPL 规划",
                   arm_type.c_str(), fraction * 100);
      // Cartesian 不完整，回退到 OMPL 规划
      bool ompl_success = (move_group.plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      plan_success = ompl_success;
      if (ompl_success) {
        RCLCPP_INFO(this->get_logger(), "%s臂 OMPL 规划成功（回退）", arm_type.c_str());
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "%s臂 Cartesian 规划完全失败，尝试 OMPL 规划",
                   arm_type.c_str());
      // Cartesian 完全失败，尝试 OMPL
      bool ompl_success = (move_group.plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      plan_success = ompl_success;
    }

    if (!plan_success) {
      RCLCPP_ERROR(this->get_logger(), "%s臂轨迹规划失败，可能位姿不可达或存在碰撞",
                   arm_type.c_str());
      result->error_code = 2;
      goal_handle->abort(result);
      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_.erase(goal_id);
      moveit_executor.cancel();
      // join() with timeout protection
      if (moveit_thread.joinable()) {
        moveit_thread.join();
      }
      return;
    }
    RCLCPP_INFO(this->get_logger(), "%s臂规划成功，耗时: %.2f秒",
                 arm_type.c_str(), motion_plan.planning_time_);

    // 手动进行时间参数化（降低速度减少抖动）
    robot_trajectory::RobotTrajectory robot_traj(move_group.getRobotModel(), move_group_name);
    robot_traj.setRobotTrajectoryMsg(*move_group.getCurrentState(), motion_plan.trajectory_);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    double velocity_scaling = (goal->velocity == 0) ? 0.1 : goal->velocity / 100.0 * 0.1;
    double acceleration_scaling = 0.1;
    bool time_param_success = totg.computeTimeStamps(robot_traj, velocity_scaling, acceleration_scaling);

    if (time_param_success) {
        robot_traj.getRobotTrajectoryMsg(motion_plan.trajectory_);
        RCLCPP_INFO(this->get_logger(), "时间参数化成功，轨迹点数: %zu",
                    motion_plan.trajectory_.joint_trajectory.points.size());
    } else {
        RCLCPP_ERROR(this->get_logger(), "时间参数化失败！");
        result->error_code = 5;
        goal_handle->abort(result);
        std::lock_guard<std::mutex> lock(map_mutex_);
        goal_arm_map_.erase(goal_id);
        moveit_executor.cancel();
        if (moveit_thread.joinable()) {
            moveit_thread.join();
        }
        return;
    }

    // 执行规划的轨迹
    std::vector<std::string> joint_names = move_group.getJoints();
    bool execute_success = js_monitor_.executeWithTimeout(move_group, motion_plan, 3.0);

    if (!execute_success) {
      RCLCPP_ERROR(this->get_logger(), "%s臂轨迹执行失败", arm_type.c_str());
      result->error_code = 3;
      goal_handle->abort(result);
      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_.erase(goal_id);
      moveit_executor.cancel();
      if (moveit_thread.joinable()) {
        moveit_thread.join();
      }
      return;
    }

    // 验证是否真的到达目标（获取最终关节位置）
    auto final_state = move_group.getCurrentState();
    std::vector<double> final_positions;
    final_state->copyJointGroupPositions(move_group_name, final_positions);

    if (!js_monitor_.verifyExecution(joint_names, final_positions)) {
      RCLCPP_ERROR(this->get_logger(), "%s臂执行验证失败，SDK可能已挂", arm_type.c_str());
      result->error_code = 3;
      goal_handle->abort(result);
      std::lock_guard<std::mutex> lock(map_mutex_);
      goal_arm_map_.erase(goal_id);
      moveit_executor.cancel();
      if (moveit_thread.joinable()) {
        moveit_thread.join();
      }
      return;
    }

    // 返回成功结果
    feedback->timestamp = this->get_clock()->now();
    feedback->process_status = 100;
    goal_handle->publish_feedback(feedback);

    result->timestamp = this->get_clock()->now();
    result->planning_time.sec = static_cast<int>(motion_plan.planning_time_);
    result->planning_time.nanosec = static_cast<int>(
      (motion_plan.planning_time_ - result->planning_time.sec) * 1e9
    );
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "%s臂末端已成功到达目标位置", arm_type.c_str());

    // 清理
    std::lock_guard<std::mutex> lock(map_mutex_);
    goal_arm_map_.erase(goal_id);
    moveit_executor.cancel();
    // join() with timeout protection
    if (moveit_thread.joinable()) {
      moveit_thread.join();
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpPositionActionServer>());
  rclcpp::shutdown();
  return 0;
}
