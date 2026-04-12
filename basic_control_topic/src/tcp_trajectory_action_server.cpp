/**
 * ================================================================================
 * TCP 轨迹规划与导出服务器 (TCP Trajectory Action Server)
 * ================================================================================
 *
 * 功能: 接收多个 TCP 动作序列，规划轨迹，并导出为 CSV 文件
 *
 * 核心:
 *   1. 支持多种运动类型：PTP（点对点）、LIN（直线）、CIRC（圆弧）
 *   2. 轨迹规划：通过 MoveIt 的 Motion Sequence 接口进行复杂序列规划
 *   3. 数据导出：轨迹保存为 CSV 格式，用于离线学习或验证
 *   4. 关节映射：自动处理 MoveIt 返回的关节顺序与期望顺序的不匹配
 *   5. 奇异点检测：规划成功后通过 Jacobian 条件数检测轨迹是否经过奇异区域
 *
 * ================================================================================
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>

#include <planning_sdk_msgs/action/tcp_trajectory.hpp>
#include <planning_sdk_msgs/msg/tcp_action.hpp>
#include <planning_sdk_msgs/msg/joint_limits_array.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <Eigen/Dense>

// ============================================================
// 类型别名
// ============================================================
using TcpTrajectory        = planning_sdk_msgs::action::TcpTrajectory;
using GoalHandlePtr        = std::shared_ptr<rclcpp_action::ServerGoalHandle<TcpTrajectory>>;
using GetMotionSequence    = moveit_msgs::srv::GetMotionSequence;
using MotionSequenceRequest  = moveit_msgs::msg::MotionSequenceRequest;
using MotionSequenceItem     = moveit_msgs::msg::MotionSequenceItem;
using MotionSequenceResponse = moveit_msgs::msg::MotionSequenceResponse;
using MoveItErrorCodes       = moveit_msgs::msg::MoveItErrorCodes;
using TcpAction              = planning_sdk_msgs::msg::TcpAction;

// ============================================================
// 常量
// ============================================================

// 奇异点检测：Jacobian 条件数阈值，超过此值认为经过奇异区域
static constexpr double SINGULARITY_COND_THRESHOLD = 100.0;

// velocity (1-100) 映射到 MoveIt scaling factor 的系数: vel_scale = velocity / 100 * VELOCITY_SCALE_FACTOR
static constexpr double VELOCITY_SCALE_FACTOR = 0.2;

// CIRC 运动的速度/加速度上限
static constexpr double CIRC_MAX_SCALE = 0.3;

// CIRC 自动中间点偏移量的默认值与上下限 (m)
static constexpr double CIRC_BULGE_RATIO_DEFAULT = 0.15;
static constexpr double CIRC_MIN_BULGE = 0.02;   // 最小偏移：短距离也能看到弧
static constexpr double CIRC_MAX_BULGE = 0.10;    // 最大偏移：长距离不会弧太大

// CIRC 自动拆分：超过此距离的 CIRC 自动拆为多段小弧 (m)
static constexpr double CIRC_MAX_SEGMENT_DIST_DEFAULT = 0.03;

// blend_radius 相关
static constexpr double BLEND_MIN_DISTANCE = 0.3;   // 相邻点距离低于此值禁用融合 (m)
static constexpr double BLEND_MAX_RADIUS   = 0.1;   // 融合半径上限 (m)
static constexpr double BLEND_DIST_RATIO   = 0.2;   // 融合半径 = min(上限, 距离 * 此比例) * cont/100

// 规划服务相关
static constexpr int    PLANNING_SERVICE_RETRIES = 5;
static constexpr int    PLANNING_SERVICE_WAIT_SEC = 2;
static constexpr int    PLANNING_TIMEOUT_SEC = 30;

// CSV 导出
static constexpr double TIME_EPSILON = 1e-6;

// ============================================================
// 臂侧配置
// ============================================================
struct ArmConfig {
  std::string planning_group;
  std::string end_effector;
  std::vector<std::string> joint_order;
};

static const std::map<std::string, ArmConfig> ARM_CONFIGS = {
  {"left", {
    "left_arm_group",
    "left_gripper_base_link",
    {
      "left_shoulder_pitch_joint",
      "left_shoulder_roll_joint",
      "left_shoulder_yaw_joint",
      "left_elbow_joint",
      "left_wrist_roll_joint",
      "left_wrist_yaw_joint",
      "left_wrist_pitch_joint"
    }
  }},
  {"right", {
    "right_arm_group",
    "right_gripper_base_link",
    {
      "right_shoulder_pitch_joint",
      "right_shoulder_roll_joint",
      "right_shoulder_yaw_joint",
      "right_elbow_joint",
      "right_wrist_roll_joint",
      "right_wrist_yaw_joint",
      "right_wrist_pitch_joint"
    }
  }}
};

// ============================================================
// TcpTrajectoryActionServer
// ============================================================
class TcpTrajectoryActionServer : public rclcpp::Node
{
public:
  TcpTrajectoryActionServer() : Node("tcp_trajectory_action_server")
  {
    // 声明可动态调节的 CIRC 参数
    this->declare_parameter("circ_bulge_ratio", CIRC_BULGE_RATIO_DEFAULT);
    this->declare_parameter("circ_min_bulge", CIRC_MIN_BULGE);
    this->declare_parameter("circ_max_bulge", CIRC_MAX_BULGE);
    this->declare_parameter("circ_max_segment_dist", CIRC_MAX_SEGMENT_DIST_DEFAULT);

    initRobotModel();

    action_server_left_ = rclcpp_action::create_server<TcpTrajectory>(
      this,
      "/algorithm/grasp_planning/move/left_arm/tcp_trajectory",
      std::bind(&TcpTrajectoryActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TcpTrajectoryActionServer::handleCancel, this, std::placeholders::_1),
      [this](GoalHandlePtr gh) { std::thread([this, gh]() { executePlan(gh, "left"); }).detach(); });

    action_server_right_ = rclcpp_action::create_server<TcpTrajectory>(
      this,
      "/algorithm/grasp_planning/move/right_arm/tcp_trajectory",
      std::bind(&TcpTrajectoryActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TcpTrajectoryActionServer::handleCancel, this, std::placeholders::_1),
      [this](GoalHandlePtr gh) { std::thread([this, gh]() { executePlan(gh, "right"); }).detach(); });

    planning_client_ = this->create_client<GetMotionSequence>("/plan_sequence_path");

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        latest_joint_state_ = *msg;
      });

    auto limits_qos = rclcpp::QoS(1).transient_local();
    joint_limits_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointLimitsArray>(
      "/algorithm/joint_limits/current", limits_qos,
      [this](const planning_sdk_msgs::msg::JointLimitsArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(soft_limits_mutex_);
        soft_limits_.clear();
        for (const auto& jl : msg->limits) {
          soft_limits_[jl.joint_name] = {jl.lower_limit, jl.upper_limit};
        }
        RCLCPP_INFO(this->get_logger(), "Updated soft limits (%zu joints)", msg->limits.size());
      });

    RCLCPP_INFO(this->get_logger(), "TCP Trajectory Action Server ready for both arms");
  }

private:
  // ---- 成员变量 ----
  rclcpp_action::Server<TcpTrajectory>::SharedPtr action_server_left_;
  rclcpp_action::Server<TcpTrajectory>::SharedPtr action_server_right_;
  rclcpp::Client<GetMotionSequence>::SharedPtr    planning_client_;
  moveit::core::RobotModelPtr                     robot_model_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  sensor_msgs::msg::JointState latest_joint_state_;
  std::mutex joint_state_mutex_;

  struct PosLimit { double min_pos; double max_pos; };
  std::map<std::string, PosLimit> soft_limits_;
  std::mutex soft_limits_mutex_;
  rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;

  // ============================================================
  // 初始化
  // ============================================================
  void initRobotModel()
  {
    auto loader = std::make_shared<robot_model_loader::RobotModelLoader>(
        std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}));
    robot_model_ = loader->getModel();
    if (!robot_model_) {
      RCLCPP_WARN(this->get_logger(), "Failed to load robot model, singularity detection disabled");
    } else {
      RCLCPP_INFO(this->get_logger(), "Robot model loaded for singularity detection");
    }
  }

  // ============================================================
  // Action 回调
  // ============================================================
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const TcpTrajectory::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received planning goal with %zu actions", goal->tcp_actions.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const GoalHandlePtr)
  {
    RCLCPP_INFO(this->get_logger(), "Goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // ============================================================
  // 主执行流程
  // ============================================================
  void executePlan(const GoalHandlePtr goal_handle, const std::string& arm_side)
  {
    const auto& arm = ARM_CONFIGS.at(arm_side);
    auto goal     = goal_handle->get_goal();
    auto result   = std::make_shared<TcpTrajectory::Result>();
    auto feedback = std::make_shared<TcpTrajectory::Feedback>();
    auto start_time = this->now();

    try {
      publishFeedback(goal_handle, feedback, 1);

      // 空动作序列
      if (goal->tcp_actions.empty()) {
        RCLCPP_WARN(this->get_logger(), "Empty action sequence received for %s arm", arm_side.c_str());
        fillResult(result, 1, false, builtin_interfaces::msg::Duration());
        goal_handle->succeed(result);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Planning for %s arm (group: %s, ee: %s)",
                  arm_side.c_str(), arm.planning_group.c_str(), arm.end_effector.c_str());

      // 等待规划服务
      waitForPlanningService();

      // 构建并发送规划请求
      auto request = std::make_shared<GetMotionSequence::Request>();
      request->request = buildSequence(goal, arm, arm_side);

      RCLCPP_INFO(this->get_logger(), "Sending planning request for %s arm with %zu items",
                  arm_side.c_str(), request->request.items.size());

      auto response = sendPlanningRequest(request, arm_side);
      logPlanningResponse(response);

      // 规划失败处理
      if (response->response.error_code.val != MoveItErrorCodes::SUCCESS) {
        bool is_no_ik = (response->response.error_code.val == MoveItErrorCodes::NO_IK_SOLUTION);
        if (is_no_ik) {
          RCLCPP_WARN(this->get_logger(), "%s arm: NO_IK_SOLUTION, marking had_singular=true", arm_side.c_str());
        }
        fillResult(result, 1, is_no_ik, this->now() - start_time);
        goal_handle->abort(result);
        return;
      }

      // 轨迹完整性校验：规划成功但没有轨迹点
      if (response->response.planned_trajectories.empty()) {
        RCLCPP_ERROR(this->get_logger(), "%s arm: planning succeeded but returned no trajectories", arm_side.c_str());
        fillResult(result, 1, false, this->now() - start_time);
        goal_handle->abort(result);
        return;
      }

      publishFeedback(goal_handle, feedback, 2);

      // 软限位验证：检查规划轨迹所有点是否在限位内
      {
        std::lock_guard<std::mutex> lock(soft_limits_mutex_);
        bool violated = false;
        for (const auto& traj : response->response.planned_trajectories) {
          for (const auto& point : traj.joint_trajectory.points) {
            for (size_t i = 0; i < traj.joint_trajectory.joint_names.size() && i < point.positions.size(); ++i) {
              const auto& jname = traj.joint_trajectory.joint_names[i];
              auto lit = soft_limits_.find(jname);
              if (lit != soft_limits_.end()) {
                if (point.positions[i] < lit->second.min_pos || point.positions[i] > lit->second.max_pos) {
                  RCLCPP_ERROR(this->get_logger(),
                    "%s arm: Trajectory REJECTED [%s] pos=%.3f not in [%.3f, %.3f]",
                    arm_side.c_str(), jname.c_str(), point.positions[i],
                    lit->second.min_pos, lit->second.max_pos);
                  violated = true;
                }
              }
            }
          }
        }
        if (violated) {
          fillResult(result, 1, false, this->now() - start_time);
          goal_handle->abort(result);
          return;
        }
        RCLCPP_INFO(this->get_logger(), "%s arm: All trajectory points passed soft limit check", arm_side.c_str());
      }

      // 奇异点检测
      bool singular = checkSingularity(response->response, arm.planning_group);
      if (singular) {
        RCLCPP_WARN(this->get_logger(), "%s arm trajectory passes through singular region!", arm_side.c_str());
      }

      // CSV 导出
      if (!goal->trajectory_file.empty()) {
        RCLCPP_INFO(this->get_logger(), "Saving %s arm trajectory to CSV: %s",
                    arm_side.c_str(), goal->trajectory_file.c_str());
        saveTrajectoryToCSV(response->response, goal->trajectory_file, arm);
      }

      publishFeedback(goal_handle, feedback, 3);

      // 提取最终关节配置
      sensor_msgs::msg::JointState end_joint_config;
      if (!response->response.planned_trajectories.empty()) {
        const auto& last_traj = response->response.planned_trajectories.back();
        if (!last_traj.joint_trajectory.points.empty()) {
          const auto& last_point = last_traj.joint_trajectory.points.back();
          end_joint_config.name = last_traj.joint_trajectory.joint_names;
          end_joint_config.position = last_point.positions;
        }
      }

      fillResult(result, 0, singular, this->now() - start_time, end_joint_config);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "%s arm trajectory planning completed successfully", arm_side.c_str());

    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error in %s arm planning: %s", arm_side.c_str(), e.what());
      fillResult(result, 1, false, this->now() - start_time);
      goal_handle->abort(result);
    }
  }

  // ============================================================
  // 辅助方法
  // ============================================================
  void publishFeedback(const GoalHandlePtr& gh,
                       std::shared_ptr<TcpTrajectory::Feedback>& fb,
                       uint16_t status)
  {
    fb->timestamp = this->now();
    fb->process_status = status;
    gh->publish_feedback(fb);
  }

  void fillResult(std::shared_ptr<TcpTrajectory::Result>& result,
                  uint16_t error_code, bool had_singular,
                  const builtin_interfaces::msg::Duration& planning_time,
                  const sensor_msgs::msg::JointState& end_joint_config = sensor_msgs::msg::JointState())
  {
    result->timestamp     = this->now();
    result->error_code    = error_code;
    result->had_singular  = had_singular;
    result->planning_time = planning_time;
    result->end_joint_configuration = end_joint_config;
  }

  // ============================================================
  // 规划服务通信
  // ============================================================
  void waitForPlanningService()
  {
    for (int i = 0; i < PLANNING_SERVICE_RETRIES; ++i) {
      if (planning_client_->wait_for_service(std::chrono::seconds(PLANNING_SERVICE_WAIT_SEC))) {
        return;
      }
      RCLCPP_WARN(this->get_logger(), "Planning service not available, retrying... (%d/%d)", i + 1, PLANNING_SERVICE_RETRIES);
    }
    throw std::runtime_error("Planning service unavailable after multiple retries");
  }

  std::shared_ptr<GetMotionSequence::Response> sendPlanningRequest(
    const std::shared_ptr<GetMotionSequence::Request>& request,
    const std::string& arm_side)
  {
    auto future = planning_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(PLANNING_TIMEOUT_SEC)) != std::future_status::ready) {
      throw std::runtime_error("Planning service call timeout for " + arm_side + " arm");
    }
    return future.get();
  }

  void logPlanningResponse(const std::shared_ptr<GetMotionSequence::Response>& response)
  {
    RCLCPP_INFO(this->get_logger(), "Planning response error code: %d", response->response.error_code.val);
    RCLCPP_INFO(this->get_logger(), "Number of planned trajectories: %zu", response->response.planned_trajectories.size());

    for (size_t i = 0; i < response->response.planned_trajectories.size(); ++i) {
      const auto& traj = response->response.planned_trajectories[i];
      RCLCPP_INFO(this->get_logger(), "Trajectory %zu: %zu points, %zu joints",
                  i, traj.joint_trajectory.points.size(), traj.joint_trajectory.joint_names.size());
    }
  }

  // ============================================================
  // 奇异点检测
  // ============================================================
  bool checkSingularity(const MotionSequenceResponse& response,
                        const std::string& planning_group)
  {
    if (!robot_model_) return false;

    const auto* jmg = robot_model_->getJointModelGroup(planning_group);
    if (!jmg) {
      RCLCPP_WARN(this->get_logger(), "Joint model group '%s' not found, skip singularity check",
                  planning_group.c_str());
      return false;
    }

    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();

    double worst_cond = 0.0;
    int singular_count = 0;

    for (const auto& traj : response.planned_trajectories) {
      const auto& names = traj.joint_trajectory.joint_names;
      for (const auto& point : traj.joint_trajectory.points) {
        for (size_t j = 0; j < names.size() && j < point.positions.size(); ++j) {
          state.setJointPositions(names[j], &point.positions[j]);
        }
        state.update();

        Eigen::MatrixXd jacobian;
        if (!state.getJacobian(jmg, jmg->getLinkModels().back(),
                               Eigen::Vector3d::Zero(), jacobian)) {
          continue;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
        double sigma_min = svd.singularValues()(svd.singularValues().size() - 1);
        double sigma_max = svd.singularValues()(0);
        double cond = (sigma_min > 1e-10) ? (sigma_max / sigma_min) : 1e10;

        worst_cond = std::max(worst_cond, cond);
        if (cond > SINGULARITY_COND_THRESHOLD) {
          singular_count++;
        }
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Singularity check for %s: worst_condition=%.1f, singular_points=%d (threshold=%.0f)",
                planning_group.c_str(), worst_cond, singular_count, SINGULARITY_COND_THRESHOLD);

    return singular_count > 0;
  }

  // ============================================================
  // 构建 MoveIt MotionSequence 请求
  // ============================================================
  MotionSequenceRequest buildSequence(
    const std::shared_ptr<const TcpTrajectory::Goal>& goal,
    const ArmConfig& arm,
    const std::string& arm_side)
  {
    MotionSequenceRequest sequence;
    const auto& tcp_actions = goal->tcp_actions;

    for (size_t i = 0; i < tcp_actions.size(); ++i) {
      const auto& action = tcp_actions[i];
      MotionSequenceItem item;

      // blend_radius
      item.blend_radius = computeBlendRadius(action, tcp_actions, i, arm_side);

      // 规划参数
      item.req.group_name = arm.planning_group;
      item.req.pipeline_id = "pilz_industrial_motion_planner";

      // 起始状态设置
      if (i == 0 && goal->is_use_start_joint_configuration) {
        // 第一个动作：使用指定的起始关节配置
        item.req.start_state.is_diff = false;
        item.req.start_state.joint_state = goal->start_joint_configuration;
        RCLCPP_INFO(this->get_logger(), "Using custom start_joint_configuration for first action");
      } else {
        // 后续动作：从前一个动作的结束状态开始
        item.req.start_state.is_diff = true;
      }

      item.req.allowed_planning_time = 10.0;

      // 速度/加速度：action.velocity * global_velocity
      // 默认值：action.velocity 默认 ，global_velocity 默认 
      double action_vel = (action.velocity > 1) ? static_cast<double>(action.velocity) : 50.0;
      double global_vel = (goal->global_velocity > 1) ? static_cast<double>(goal->global_velocity) : 90.0;

      double vel_scale = (action_vel / 100.0) * (global_vel / 100.0) * VELOCITY_SCALE_FACTOR;
      double acc_scale = vel_scale;
      if (action.action == 2) {
        vel_scale = std::min(vel_scale, CIRC_MAX_SCALE);
        acc_scale = std::min(acc_scale, CIRC_MAX_SCALE);
      }
      item.req.max_velocity_scaling_factor     = vel_scale;
      item.req.max_acceleration_scaling_factor  = acc_scale;

      RCLCPP_INFO(this->get_logger(), "Action[%zu] velocity=%.0f, global_velocity=%.0f -> vel_scale=%.4f, acc_scale=%.4f",
                  i, action_vel, global_vel, vel_scale, acc_scale);

      // planner_id
      item.req.planner_id = actionToPlannerId(action.action, arm_side);

      // PTP：添加关节软限位路径约束
      if (action.action == 0) {
        std::lock_guard<std::mutex> lock(soft_limits_mutex_);
        if (!soft_limits_.empty()) {
          for (const auto& jname : arm.joint_order) {
            auto lit = soft_limits_.find(jname);
            if (lit != soft_limits_.end()) {
              double center = (lit->second.min_pos + lit->second.max_pos) / 2.0;
              moveit_msgs::msg::JointConstraint jc;
              jc.joint_name = jname;
              jc.position = center;
              jc.tolerance_above = lit->second.max_pos - center;
              jc.tolerance_below = center - lit->second.min_pos;
              jc.weight = 1.0;
              item.req.path_constraints.joint_constraints.push_back(jc);
            }
          }
          RCLCPP_INFO(this->get_logger(), "Action[%zu] PTP: added %zu joint path constraints",
                      i, item.req.path_constraints.joint_constraints.size());
        }
      }

      // 目标约束
      item.req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(arm.end_effector, action.target_point));

      // CIRC 中间点约束（大弧自动拆分）
      if (action.action == 2) {
        geometry_msgs::msg::PoseStamped start_pose;
        if (i > 0) {
          start_pose = tcp_actions[i - 1].target_point;
        } else {
          start_pose = getCurrentEEPose(arm);
        }

        if (!action.waypoints.empty()) {
          // 用户指定了 waypoint，单段 CIRC
          item.req.path_constraints = buildCircConstraints(action.waypoints[0], arm.end_effector);
          RCLCPP_INFO(this->get_logger(), "CIRC motion for %s arm - user-specified interim: (%.3f, %.3f, %.3f) in frame %s",
                      arm_side.c_str(),
                      action.waypoints[0].pose.position.x,
                      action.waypoints[0].pose.position.y,
                      action.waypoints[0].pose.position.z,
                      action.waypoints[0].header.frame_id.c_str());
        } else {
          // 自动模式：始终使用原生 CIRC（临时禁用拆分，验证 Pilz CIRC 可行性）
          // Eigen::Vector3d ps(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
          // Eigen::Vector3d pe(action.target_point.pose.position.x, action.target_point.pose.position.y, action.target_point.pose.position.z);
          // double dist = (pe - ps).norm();
          //
          // double max_seg = this->get_parameter("circ_max_segment_dist").as_double();
          // if (dist > max_seg) {
          //   auto segments = buildCircSegments(start_pose, action.target_point,
          //                                     vel_scale, acc_scale,
          //                                     item.blend_radius, arm, arm_side);
          //   for (auto& seg : segments) {
          //     sequence.items.push_back(std::move(seg));
          //   }
          //   continue;  // 已添加多段，跳过下面的单段 push_back
          // }

          // 单段原生 CIRC
          auto interim = computeCircInterim(start_pose, action.target_point, arm_side);
          item.req.path_constraints = buildCircConstraints(interim, arm.end_effector);
          RCLCPP_INFO(this->get_logger(), "CIRC motion for %s arm - auto-computed interim: (%.3f, %.3f, %.3f)",
                      arm_side.c_str(),
                      interim.pose.position.x,
                      interim.pose.position.y,
                      interim.pose.position.z);
        }
      }

      sequence.items.push_back(item);
    }

    RCLCPP_INFO(this->get_logger(), "Created sequence with %zu items for %s arm",
                sequence.items.size(), arm_side.c_str());
    return sequence;
  }

  std::string actionToPlannerId(uint16_t action_type, const std::string& arm_side)
  {
    switch (action_type) {
      case 0:  RCLCPP_INFO(this->get_logger(), "Adding PTP action for %s arm", arm_side.c_str());  return "PTP";
      case 1:  RCLCPP_INFO(this->get_logger(), "Adding LIN action for %s arm", arm_side.c_str());  return "LIN";
      case 2:  RCLCPP_INFO(this->get_logger(), "Adding CIRC action for %s arm", arm_side.c_str()); return "CIRC";
      default: throw std::invalid_argument("Unknown action type for " + arm_side + " arm: " + std::to_string(action_type));
    }
  }

  // ============================================================
  // blend_radius 计算
  // ============================================================
  double computeBlendRadius(const TcpAction& action,
                            const std::vector<TcpAction>& actions,
                            size_t index,
                            const std::string& arm_side)
  {
    // 最后一个点或 cont=0：不融合
    if (index == actions.size() - 1) return 0.0;
    if (action.cont == 0) {
      RCLCPP_INFO(this->get_logger(), "Action[%zu] cont=0, blend disabled (exact stop)", index);
      return 0.0;
    }

    double cont_ratio = std::clamp(static_cast<double>(action.cont), 0.0, 100.0) / 100.0;

    const auto& p1 = action.target_point.pose.position;
    const auto& p2 = actions[index + 1].target_point.pose.position;
    double dx = p2.x - p1.x, dy = p2.y - p1.y, dz = p2.z - p1.z;
    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

    if (dist < BLEND_MIN_DISTANCE) {
      RCLCPP_INFO(this->get_logger(), "Action[%zu] -> Action[%zu] distance: %.4f m (< %.1fm), blend disabled despite cont=%u",
                  index, index + 1, dist, BLEND_MIN_DISTANCE, action.cont);
      return 0.0;
    }

    double radius = std::min(BLEND_MAX_RADIUS, dist * BLEND_DIST_RATIO) * cont_ratio;
    RCLCPP_INFO(this->get_logger(), "Action[%zu] -> Action[%zu] distance: %.4f m, cont=%u, blend_radius: %.4f m",
                index, index + 1, dist, action.cont, radius);
    return radius;
  }

  // ============================================================
  // CIRC 自动中间点
  // ============================================================

  // 根据起终点距离，动态计算弧线偏移量（比例 + 上下限钳位）
  double computeBulge(double dist)
  {
    double ratio = this->get_parameter("circ_bulge_ratio").as_double();
    double min_b = this->get_parameter("circ_min_bulge").as_double();
    double max_b = this->get_parameter("circ_max_bulge").as_double();
    double bulge = std::clamp(dist * ratio, min_b, max_b);
    RCLCPP_INFO(this->get_logger(),
                "CIRC bulge: dist=%.3fm, ratio=%.3f, raw=%.4fm, clamped=[%.3f,%.3f] -> %.4fm",
                dist, ratio, dist * ratio, min_b, max_b, bulge);
    return bulge;
  }

  // 计算弧线偏移方向：类人臂优先向上(+Z)弧形，垂直运动时根据臂侧向外偏移
  Eigen::Vector3d computeArcDirection(const Eigen::Vector3d& dir, const std::string& arm_side)
  {
    Eigen::Vector3d dir_n = dir.normalized();

    // 将 +Z（向上）投影到垂直于运动方向的平面：对水平/斜向运动产生向上弧形
    Eigen::Vector3d up(0, 0, 1);
    Eigen::Vector3d perp = up - dir_n * dir_n.dot(up);

    if (perp.norm() > 1e-6) {
      RCLCPP_INFO(this->get_logger(), "CIRC arc direction: upward (+Z projected), %s arm", arm_side.c_str());
      return perp.normalized();
    }

    // 运动方向接近垂直时，根据臂侧选择外侧方向避免碰撞
    Eigen::Vector3d outward = (arm_side == "left") ? Eigen::Vector3d(0, 1, 0) : Eigen::Vector3d(0, -1, 0);
    RCLCPP_INFO(this->get_logger(), "CIRC arc direction: outward (%.0f,%.0f,%.0f), %s arm (vertical motion)",
                outward.x(), outward.y(), outward.z(), arm_side.c_str());
    return outward;
  }

  // 通过 FK 获取当前末端执行器位姿
  geometry_msgs::msg::PoseStamped getCurrentEEPose(const ArmConfig& arm)
  {
    if (!robot_model_) {
      throw std::runtime_error("Robot model not loaded, cannot compute current EE pose");
    }

    moveit::core::RobotState state(robot_model_);
    state.setToDefaultValues();

    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (latest_joint_state_.name.empty()) {
        throw std::runtime_error("No joint state received yet, cannot compute current EE pose for CIRC");
      }
      for (size_t i = 0; i < latest_joint_state_.name.size() && i < latest_joint_state_.position.size(); ++i) {
        state.setJointPositions(latest_joint_state_.name[i], &latest_joint_state_.position[i]);
      }
    }
    state.update();

    const auto& tf = state.getGlobalLinkTransform(arm.end_effector);
    Eigen::Quaterniond q(tf.rotation());

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "base_link";
    pose.pose.position.x = tf.translation().x();
    pose.pose.position.y = tf.translation().y();
    pose.pose.position.z = tf.translation().z();
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    return pose;
  }

  // 根据起点和终点自动计算 CIRC 中间点（向上弧形，避免碰撞）
  geometry_msgs::msg::PoseStamped computeCircInterim(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& end,
    const std::string& arm_side)
  {
    Eigen::Vector3d p0(start.pose.position.x, start.pose.position.y, start.pose.position.z);
    Eigen::Vector3d p1(end.pose.position.x, end.pose.position.y, end.pose.position.z);

    Eigen::Vector3d mid = (p0 + p1) * 0.5;
    Eigen::Vector3d dir = p1 - p0;
    double dist = dir.norm();

    Eigen::Vector3d perp = computeArcDirection(dir, arm_side);

    // 中间点 = 中点 + 垂直偏移（动态计算偏移量，带上下限钳位）
    Eigen::Vector3d interim_pos = mid + perp * computeBulge(dist);

    // 姿态：起终点 SLERP 插值
    Eigen::Quaterniond q0(start.pose.orientation.w, start.pose.orientation.x,
                          start.pose.orientation.y, start.pose.orientation.z);
    Eigen::Quaterniond q1(end.pose.orientation.w, end.pose.orientation.x,
                          end.pose.orientation.y, end.pose.orientation.z);
    Eigen::Quaterniond qi = q0.slerp(0.5, q1);

    geometry_msgs::msg::PoseStamped interim;
    interim.header.frame_id = start.header.frame_id;
    interim.pose.position.x = interim_pos.x();
    interim.pose.position.y = interim_pos.y();
    interim.pose.position.z = interim_pos.z();
    interim.pose.orientation.x = qi.x();
    interim.pose.orientation.y = qi.y();
    interim.pose.orientation.z = qi.z();
    interim.pose.orientation.w = qi.w();
    return interim;
  }

  // 将大 CIRC 拆分为多段 LIN（沿弧线采样，保持圆弧路径形状）
  // 注意：不使用 PTP，因为 PTP 是关节空间插值，无法保证笛卡尔路径呈弧形
  std::vector<MotionSequenceItem> buildCircSegments(
    const geometry_msgs::msg::PoseStamped& start_pose,
    const geometry_msgs::msg::PoseStamped& end_pose,
    double vel_scale, double acc_scale,
    double final_blend_radius,
    const ArmConfig& arm,
    const std::string& arm_side)
  {
    std::vector<MotionSequenceItem> items;

    Eigen::Vector3d p0(start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
    Eigen::Vector3d p1(end_pose.pose.position.x, end_pose.pose.position.y, end_pose.pose.position.z);
    Eigen::Quaterniond q0(start_pose.pose.orientation.w, start_pose.pose.orientation.x,
                          start_pose.pose.orientation.y, start_pose.pose.orientation.z);
    Eigen::Quaterniond q1(end_pose.pose.orientation.w, end_pose.pose.orientation.x,
                          end_pose.pose.orientation.y, end_pose.pose.orientation.z);

    double dist = (p1 - p0).norm();
    double max_seg = this->get_parameter("circ_max_segment_dist").as_double();
    int N = std::max(2, static_cast<int>(std::ceil(dist / max_seg)));

    // 计算弧线中间点（使用安全方向：优先向上）
    Eigen::Vector3d mid_linear = (p0 + p1) * 0.5;
    Eigen::Vector3d dir = p1 - p0;
    Eigen::Vector3d perp = computeArcDirection(dir, arm_side);
    Eigen::Vector3d pm = mid_linear + perp * computeBulge(dist);

    RCLCPP_INFO(this->get_logger(),
                "Splitting large CIRC into %d LIN segments for %s arm (total: %.3fm, arc mid: (%.3f,%.3f,%.3f))",
                N, arm_side.c_str(), dist, pm.x(), pm.y(), pm.z());

    std::string frame_id = end_pose.header.frame_id;
    Eigen::Vector3d prev_pos = p0;  // 上一段终点，用于计算段长

    for (int seg = 0; seg < N; ++seg) {
      double t = static_cast<double>(seg + 1) / N;

      // 三点 Lagrange 插值沿弧线采样（t=0: p0, t=0.5: pm, t=1: p1）
      double w0 = 2.0 * (t - 0.5) * (t - 1.0);
      double wm = -4.0 * t * (t - 1.0);
      double w1 = 2.0 * t * (t - 0.5);
      Eigen::Vector3d ep = p0 * w0 + pm * wm + p1 * w1;
      Eigen::Quaterniond eq = q0.slerp(t, q1);

      double seg_len = (ep - prev_pos).norm();
      prev_pos = ep;

      geometry_msgs::msg::PoseStamped seg_end;
      seg_end.header.frame_id = frame_id;
      seg_end.pose.position.x = ep.x(); seg_end.pose.position.y = ep.y(); seg_end.pose.position.z = ep.z();
      seg_end.pose.orientation.x = eq.x(); seg_end.pose.orientation.y = eq.y();
      seg_end.pose.orientation.z = eq.z(); seg_end.pose.orientation.w = eq.w();

      MotionSequenceItem item;
      item.req.group_name      = arm.planning_group;
      item.req.pipeline_id     = "pilz_industrial_motion_planner";
      item.req.planner_id      = "LIN";
      item.req.start_state.is_diff = true;
      item.req.allowed_planning_time = 10.0;
      item.req.max_velocity_scaling_factor    = vel_scale;
      item.req.max_acceleration_scaling_factor = acc_scale;

      item.req.goal_constraints.push_back(
        kinematic_constraints::constructGoalConstraints(arm.end_effector, seg_end));

      // 子段间不融合（7-DOF 冗余臂 IK 构型不唯一，Pilz 无法保证关节连续性）
      // 通过细分段（默认 0.03m/段）补偿，微停顿不可感知
      if (seg < N - 1) {
        item.blend_radius = 0.0;
      } else {
        item.blend_radius = final_blend_radius;
      }

      RCLCPP_INFO(this->get_logger(), "  LIN seg %d/%d: len=%.4fm target=(%.3f,%.3f,%.3f) blend=%.4f",
                  seg + 1, N, seg_len, ep.x(), ep.y(), ep.z(), item.blend_radius);

      items.push_back(item);
    }

    return items;
  }

  // ============================================================
  // CIRC 约束
  // ============================================================
  moveit_msgs::msg::Constraints buildCircConstraints(
    const geometry_msgs::msg::PoseStamped& mid_point,
    const std::string& end_effector)
  {
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint pos_constraint;

    pos_constraint.header.frame_id = mid_point.header.frame_id;
    pos_constraint.link_name = end_effector;

    shape_msgs::msg::SolidPrimitive sphere;
    sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
    sphere.dimensions = {10.0};

    pos_constraint.constraint_region.primitives.push_back(sphere);
    pos_constraint.constraint_region.primitive_poses.push_back(mid_point.pose);
    pos_constraint.weight = 1.0;

    constraints.position_constraints.push_back(pos_constraint);
    constraints.name = "interim";
    return constraints;
  }

  // ============================================================
  // CSV 导出
  // ============================================================
  void saveTrajectoryToCSV(const MotionSequenceResponse& response,
                           const std::string& file_path,
                           const ArmConfig& arm)
  {
    // 自动创建父目录
    auto parent = std::filesystem::path(file_path).parent_path();
    if (!parent.empty() && !std::filesystem::exists(parent)) {
      std::filesystem::create_directories(parent);
      RCLCPP_INFO(this->get_logger(), "Created directory: %s", parent.c_str());
    }

    std::ofstream file(file_path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open CSV file: " + file_path);
    }

    // 表头
    file << "time";
    for (const auto& name : arm.joint_order) {
      file << "," << name;
    }
    file << "\n";

    int total_points = 0;
    double cumulative_time = 0.0;
    double last_written_time = -1.0;

    for (const auto& trajectory : response.planned_trajectories) {
      const auto& joint_names = trajectory.joint_trajectory.joint_names;
      const auto& points = trajectory.joint_trajectory.points;

      RCLCPP_INFO(this->get_logger(), "Processing trajectory with %zu points, %zu joints",
                  points.size(), joint_names.size());

      // 建立关节名 -> 索引映射
      auto desired_indices = buildJointIndexMap(joint_names, arm.joint_order);

      for (const auto& point : points) {
        double point_time = cumulative_time +
                            point.time_from_start.sec +
                            point.time_from_start.nanosec * 1e-9;

        if (point_time <= last_written_time) {
          RCLCPP_WARN(this->get_logger(),
                      "Non-increasing time detected at point %d: %.6f <= %.6f, adjusting",
                      total_points, point_time, last_written_time);
          point_time = last_written_time + TIME_EPSILON;
        }
        last_written_time = point_time;

        file << std::fixed << std::setprecision(6) << point_time;
        for (int index : desired_indices) {
          file << ",";
          if (index >= 0 && index < static_cast<int>(point.positions.size())) {
            file << std::fixed << std::setprecision(6) << point.positions[index];
          } else {
            file << "0.0";
          }
        }
        file << "\n";
        total_points++;
      }

      if (!points.empty()) {
        const auto& last_point = points.back();
        cumulative_time += last_point.time_from_start.sec +
                           last_point.time_from_start.nanosec * 1e-9;
      }
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Saved %d trajectory points to CSV: %s", total_points, file_path.c_str());

    if (total_points == 0) {
      RCLCPP_WARN(this->get_logger(), "No trajectory points were saved to CSV file!");
    }
  }

  std::vector<int> buildJointIndexMap(
    const std::vector<std::string>& actual_names,
    const std::vector<std::string>& desired_order)
  {
    std::map<std::string, int> name_to_index;
    for (size_t i = 0; i < actual_names.size(); ++i) {
      name_to_index[actual_names[i]] = static_cast<int>(i);
    }

    std::vector<int> indices;
    indices.reserve(desired_order.size());
    for (const auto& name : desired_order) {
      auto it = name_to_index.find(name);
      indices.push_back(it != name_to_index.end() ? it->second : -1);
    }
    return indices;
  }
};

// ============================================================
// main
// ============================================================
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TcpTrajectoryActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
