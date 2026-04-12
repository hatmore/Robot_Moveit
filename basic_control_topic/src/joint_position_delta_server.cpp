/**
 * ================================================================================
 * 关节位置增量控制服务器 (Joint Position Delta Action Server)
 * ================================================================================
 *
 * 功能: 接收来自客户端的关节增量移动请求，通过 MoveIt 规划和执行相对位移运动
 *
 * 核心概念:
 *   - "增量": 相对于当前位置的偏移量（单位: 弧度）
 *   - "关节组": left_arm_group 或 right_arm_group
 *   - "部分更新": 用户可以只指定部分关节的增量，其他关节保持原位置
 *
 * 调用流程:
 *   1. Client 发送 Goal (关节名列表 + 增量值列表)
 *   2. handle_goal() -> 三层验证
 *   3. handle_accepted() -> 创建独立线程
 *   4. execute() -> 计算目标位置 -> MoveIt 规划 -> 执行运动
 *
 * ================================================================================
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "planning_sdk_msgs/action/joint_position_delta.hpp"
#include "planning_sdk_msgs/msg/joint_limits_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include <functional>
#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>
#include <algorithm>
#include <future>
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "basic_control_topic/joint_state_monitor.hpp"

using JointPositionDelta = planning_sdk_msgs::action::JointPositionDelta;
using GoalHandleJointPositionDelta = rclcpp_action::ServerGoalHandle<JointPositionDelta>;
using namespace std::placeholders;

class JointPositionDeltaActionServer : public rclcpp::Node {
public:
  /**
   * 构造函数: 初始化 Action 服务器和订阅
   *
   * 初始化过程:
   *   1. 订阅 /joint_states 话题 (实时接收关节状态)
   *   2. 创建 Action 服务器 (监听客户端请求)
   *   3. 绑定三个回调函数
   *
   * 调用时机: 程序启动时 (main 中 rclcpp::init 之后)
   * 输入: 无
   * 输出: 初始化完成，开始等待请求
   */
  JointPositionDeltaActionServer() : Node("joint_position_delta_server"), js_monitor_(this) {
    // 订阅 /joint_states 话题，实时更新关节位置
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&JointPositionDeltaActionServer::jointStateCallback, this, _1));

    // 订阅动态关节限位
    auto limits_qos = rclcpp::QoS(1).transient_local();
    joint_limits_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointLimitsArray>(
      "/algorithm/joint_limits/current", limits_qos,
      std::bind(&JointPositionDeltaActionServer::jointLimitsCallback, this, _1));

    // 创建 Action 服务器
    action_server_ = rclcpp_action::create_server<JointPositionDelta>(
      this,
      "/algorithm/grasp_planning/move/joint_position_delta",
      std::bind(&JointPositionDeltaActionServer::handle_goal, this, _1, _2),
      std::bind(&JointPositionDeltaActionServer::handle_cancel, this, _1),
      std::bind(&JointPositionDeltaActionServer::handle_accepted, this, _1)
    );

    RCLCPP_INFO(this->get_logger(), "关节差值移动 Action 服务器已启动");
    RCLCPP_INFO(this->get_logger(), "name数组与position_delta数组必须长度一致、一一对应！");
  }

private:
  rclcpp_action::Server<JointPositionDelta>::SharedPtr action_server_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;
  std::unordered_map<std::string, double> current_joint_pos_map_;
  bool joint_state_received_ = false;
  std::mutex joint_state_mutex_;
  std::mutex execute_mutex_;

  // 动态关节限位
  struct PosLimit { double min_pos; double max_pos; };
  std::unordered_map<std::string, PosLimit> joint_pos_limits_;
  std::mutex limits_mutex_;
  JointStateMonitor js_monitor_;

  void jointLimitsCallback(const planning_sdk_msgs::msg::JointLimitsArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(limits_mutex_);
    joint_pos_limits_.clear();
    for (const auto& jl : msg->limits) {
      joint_pos_limits_[jl.joint_name] = {jl.lower_limit, jl.upper_limit};
    }
    RCLCPP_INFO(this->get_logger(), "Updated joint position limits (%zu joints)", msg->limits.size());
  }


  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      current_joint_pos_map_[msg->name[i]] = msg->position[i];
    }
    if (!joint_state_received_) {
      joint_state_received_ = !current_joint_pos_map_.empty();
    }
  }

  /**
   * 提取关节名称列表（当前是简单的复制操作，保留扩展可能性）
   *
   * 调用时机: handle_goal() 验证时调用
   * 输入: name_msg_array - 客户端发送的关节名称数组
   * 输出: 返回相同的关节名称向量
   *
   * 例子:
   *   输入: ["left_shoulder", "left_elbow"]
   *   输出: ["left_shoulder", "left_elbow"]
   */
  std::vector<std::string> extractJointNames(const std::vector<std::string>& name_msg_array) {
    std::vector<std::string> joint_names;
    for (const auto& name_msg : name_msg_array) {
      joint_names.push_back(name_msg);
    }
    return joint_names;
  }

  bool identifyJointGroup(const std::vector<std::string>& joint_names, std::string& joint_group_name) {
    if (joint_names.empty()) return false;

    // 步骤 1: 根据第一个关节名判断手臂
    const std::string& first_joint = joint_names[0];
    if (first_joint.find("left_") != std::string::npos) {
      joint_group_name = "left_arm_group";
    } else if (first_joint.find("right_") != std::string::npos) {
      joint_group_name = "right_arm_group";
    } else {
      RCLCPP_ERROR(this->get_logger(), "关节名[%s]无'left_'或'right_'前缀，无法识别关节组！", first_joint.c_str());
      return false;
    }

    // 步骤 2: 验证所有关节都属于同一手臂（一致性检查）
    for (const auto& joint_name : joint_names) {
      bool is_left = (joint_name.find("left_") != std::string::npos);
      bool is_right = (joint_name.find("right_") != std::string::npos);
      bool match_group = (joint_group_name == "left_arm_group" && is_left) ||
                        (joint_group_name == "right_arm_group" && is_right);

      if (!match_group) {
        RCLCPP_ERROR(this->get_logger(), "关节名[%s]与关节组[%s]不匹配！",
                     joint_name.c_str(), joint_group_name.c_str());
        return false;
      }
    }
    return true;
  }

 
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const JointPositionDelta::Goal> goal) {
    (void)uuid;

    // 第一层验证: 数组长度匹配
    if (goal->name.size() != goal->position_delta.size()) {
      RCLCPP_ERROR(this->get_logger(), "关节名数组长度（%zu）与增量数组长度（%zu）不匹配！",
                   goal->name.size(), goal->position_delta.size());
      return rclcpp_action::GoalResponse::REJECT;
    }

    // 加锁读取关节状态
    std::vector<std::string> joint_names = extractJointNames(goal->name);
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!joint_state_received_) {
        RCLCPP_ERROR(this->get_logger(), "未收到关节状态，拒绝请求！");
        return rclcpp_action::GoalResponse::REJECT;
      }

      // 第二层验证: 关节存在性
      for (const auto& joint_name : joint_names) {
        if (current_joint_pos_map_.find(joint_name) == current_joint_pos_map_.end()) {
          RCLCPP_ERROR(this->get_logger(), "关节名[%s]未找到！", joint_name.c_str());
          return rclcpp_action::GoalResponse::REJECT;
        }
      }
    }

    // 第三层验证: 语义一致性（识别关节组）
    std::string joint_group_name;
    if (!identifyJointGroup(joint_names, joint_group_name)) {
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "收到有效请求：控制关节组[%s]的%d个关节",
                 joint_group_name.c_str(), static_cast<int>(joint_names.size()));
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

 
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "收到取消请求，正在终止运动...");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

 
  void handle_accepted(const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle) {
    // 创建独立线程执行 execute() 函数
    // std::bind 绑定成员函数和参数
    std::thread{std::bind(&JointPositionDeltaActionServer::execute, this, goal_handle)}.detach();
  }

 
  void execute(const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle) {
    // 串行化执行，防止并发请求导致关节回退
    std::lock_guard<std::mutex> exec_lock(execute_mutex_);

    // ============ 阶段 1: 初始化 ============
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<JointPositionDelta::Feedback>();
    auto result = std::make_shared<JointPositionDelta::Result>();
    result->timestamp = this->get_clock()->now();

    // 发送初始反馈（0% 进度）
    feedback->timestamp = this->get_clock()->now();
    feedback->process_status = 0;
    goal_handle->publish_feedback(feedback);

    // 提取用户请求的关节名和增量
    std::vector<std::string> request_joint_names = extractJointNames(goal->name);
    const std::vector<float>& position_delta = goal->position_delta;

    // 识别关节组
    std::string joint_group_name;
    identifyJointGroup(request_joint_names, joint_group_name);

    // 创建 MoveIt 独立 Executor（关键：避免死锁）
    // 原因: 主 Executor 在 rclcpp::spin() 中运行，如果同时调用 MoveIt 的
    // 阻塞 API（plan/execute），而 MoveIt 需要处理自己的 Subscription，
    // 就会形成死锁。独立 Executor 在独立线程中运行，可以并行处理。
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto moveit_node = rclcpp::Node::make_shared("moveit_delta_node", node_options);
    rclcpp::executors::SingleThreadedExecutor moveit_executor;
    moveit_executor.add_node(moveit_node);
    std::thread moveit_thread([&moveit_executor]() { moveit_executor.spin(); });

    // ============ 阶段 2: 计算目标位置 ============
    // 获取关节组的所有关节（来自 MoveIt 配置）
    moveit::planning_interface::MoveGroupInterface move_group(moveit_node, joint_group_name);
    move_group.setPlanningTime(10.0);
    move_group.setGoalJointTolerance(0.01);
    double vel_scale = (goal->velocity == 0) ? 0.3 : goal->velocity / 100.0 * 0.5;
    move_group.setMaxVelocityScalingFactor(vel_scale);
    move_group.setMaxAccelerationScalingFactor(vel_scale);
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");  
    move_group.setPlannerId("PTP");

    std::vector<std::string> all_group_joints = move_group.getJoints();
    if (all_group_joints.empty()) {
      RCLCPP_ERROR(this->get_logger(), "关节组[%s]无关节配置！", joint_group_name.c_str());
      result->error_code = 4;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    // 构建增量映射表（关节名 -> 增量值）
    // 用于 O(1) 查询用户是否指定了某个关节的增量
    std::unordered_map<std::string, double> request_joint_delta_map;
    for (size_t i = 0; i < request_joint_names.size(); ++i) {
      request_joint_delta_map[request_joint_names[i]] = static_cast<double>(position_delta[i]);
    }

    // 计算目标位置向量
    // 在锁内拍摄关节状态快照，避免计算过程中状态被更新
    std::unordered_map<std::string, double> joint_pos_snapshot;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      joint_pos_snapshot = current_joint_pos_map_;
    }

    std::vector<double> target_positions;
    bool target_valid = true;
    for (const auto& joint : all_group_joints) {
      if (joint_pos_snapshot.find(joint) == joint_pos_snapshot.end()) {
        RCLCPP_ERROR(this->get_logger(), "关节组[%s]的关节[%s]未找到当前状态！",
                     joint_group_name.c_str(), joint.c_str());
        target_valid = false;
        break;
      }

      double current_pos = joint_pos_snapshot[joint];

      // 查询用户是否指定了增量（O(1)）
      auto delta_it = request_joint_delta_map.find(joint);
      if (delta_it != request_joint_delta_map.end()) {
        // Case A: 用户指定了增量
        double move_amount = delta_it->second;
        target_positions.push_back(current_pos + move_amount);

        if (std::fabs(move_amount) > 1e-6) {
          RCLCPP_INFO(this->get_logger(), "【关节移动】关节名：%s | 移动量：%.3f rad | 当前位置：%.3f rad | 目标位置：%.3f rad",
                      joint.c_str(), move_amount, current_pos, target_positions.back());
        } else {
          RCLCPP_INFO(this->get_logger(), "【关节不动】关节名：%s | 移动量：0.000 rad（保持当前位置：%.3f rad）",
                      joint.c_str(), current_pos);
        }
      } else {
        // Case B: 用户未指定，保持当前位置
        target_positions.push_back(current_pos);
        RCLCPP_INFO(this->get_logger(), "【关节保持】关节名：%s | 保持当前位置：%.3f rad",
                    joint.c_str(), current_pos);
      }
    }

    if (!target_valid) {
      result->error_code = 5;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    // Verify target positions are within soft limits
    {
      std::lock_guard<std::mutex> llock(limits_mutex_);
      for (size_t i = 0; i < all_group_joints.size() && i < target_positions.size(); ++i) {
        auto lit = joint_pos_limits_.find(all_group_joints[i]);
        if (lit != joint_pos_limits_.end()) {
          double pos = target_positions[i];
          if (pos < lit->second.min_pos || pos > lit->second.max_pos) {
            RCLCPP_ERROR(this->get_logger(), "Target REJECTED [%s] pos=%.3f not in [%.3f, %.3f]",
                        all_group_joints[i].c_str(), pos, lit->second.min_pos, lit->second.max_pos);
            result->error_code = 5;
            goal_handle->abort(result);
            moveit_executor.cancel();
            moveit_thread.join();
            return;
          }
        }
      }
    }

    // Add path constraints based on soft limits
    moveit_msgs::msg::Constraints joint_constraints;
    {
      std::lock_guard<std::mutex> llock(limits_mutex_);
      for (size_t i = 0; i < all_group_joints.size(); ++i) {
        auto lit = joint_pos_limits_.find(all_group_joints[i]);
        if (lit != joint_pos_limits_.end()) {
          double center = (lit->second.min_pos + lit->second.max_pos) / 2.0;
          moveit_msgs::msg::JointConstraint jc;
          jc.joint_name = all_group_joints[i];
          jc.position = center;
          jc.tolerance_above = lit->second.max_pos - center;
          jc.tolerance_below = center - lit->second.min_pos;
          jc.weight = 1.0;
          joint_constraints.joint_constraints.push_back(jc);
        }
      }
    }
    move_group.setPathConstraints(joint_constraints);
    RCLCPP_INFO(this->get_logger(), "Set %zu joint path constraints", joint_constraints.joint_constraints.size());

    // ============ 阶段 3: 路径规划 ============
    // 向 MoveIt 设置目标关节值
    // 注意: all_group_joints 和 target_positions 的顺序必须对应！
    move_group.setJointValueTarget(all_group_joints, target_positions);

    // 发送反馈：规划中（30%）
    feedback->timestamp = this->get_clock()->now();
    feedback->process_status = 30;
    goal_handle->publish_feedback(feedback);

    // 调用 MoveIt 规划接口
    // 返回值: SUCCESS (=1) 或其他错误码
    // 耗时: 1-5 秒（取决于规划复杂度和时间限制）
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success = (move_group.plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!plan_success) {
      RCLCPP_ERROR(this->get_logger(), "关节组[%s]规划失败！", joint_group_name.c_str());
      result->error_code = 2;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    // 发送反馈：规划完成（50%）
    feedback->timestamp = this->get_clock()->now();
    feedback->process_status = 50;
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "关节组[%s]规划成功，耗时：%.2f秒",
                 joint_group_name.c_str(), motion_plan.planning_time_);

    // ============ 阶段 4: 执行运动 ============
    // 移动汇总（仅显示有增量的关节）
    std::string move_summary = "【移动汇总】关节组" + joint_group_name + "中，以下关节将移动：\n";
    bool has_move_joint = false;
    for (const auto& [joint, delta] : request_joint_delta_map) {
      if (std::fabs(delta) > 1e-6) {
        move_summary += "  - " + joint + "：" + std::to_string(delta) + " rad\n";
        has_move_joint = true;
      }
    }
    if (!has_move_joint) {
      move_summary += "  - 无关节移动（所有增量均为0）\n";
    }
    RCLCPP_INFO(this->get_logger(), "%s", move_summary.c_str());

    // 检查用户是否发送了取消请求
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "运动被取消！");
      result->error_code = 1;
      goal_handle->canceled(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    // 执行规划好的轨迹，带超时控制
    bool execute_success = js_monitor_.executeWithTimeout(move_group, motion_plan, 3.0);

    if (!execute_success) {
      RCLCPP_ERROR(this->get_logger(), "关节组[%s]执行失败！", joint_group_name.c_str());
      result->error_code = 3;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    // 验证是否真的到达目标
    if (!js_monitor_.verifyExecution(all_group_joints, target_positions)) {
      RCLCPP_ERROR(this->get_logger(), "关节组[%s]执行验证失败，SDK可能已挂", joint_group_name.c_str());
      result->error_code = 3;
      goal_handle->abort(result);
      moveit_executor.cancel();
      moveit_thread.join();
      return;
    }

    // ============ 阶段 5: 清理和返回结果 ============
    // 发送最终反馈（100%）
    feedback->timestamp = this->get_clock()->now();
    feedback->process_status = 100;
    goal_handle->publish_feedback(feedback);

    // 设置结果
    result->error_code = 0;  // 成功
    result->planning_time.sec = static_cast<int32_t>(motion_plan.planning_time_);
    result->planning_time.nanosec = static_cast<uint32_t>(
      (motion_plan.planning_time_ - result->planning_time.sec) * 1e9
    );

    // 通知客户端成功
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "关节组[%s]按差值移动成功！", joint_group_name.c_str());

    // 清理 MoveIt Executor
    moveit_executor.cancel();  // 停止 spin()
    moveit_thread.join();      // 等待线程完成（通常 <1ms）
    // 函数返回后，goal_handle shared_ptr 自动析构
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto action_server_node = std::make_shared<JointPositionDeltaActionServer>();
  rclcpp::spin(action_server_node);
  rclcpp::shutdown();
  return 0;
}
