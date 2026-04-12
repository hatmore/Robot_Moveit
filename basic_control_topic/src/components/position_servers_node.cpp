/**
 * ================================================================================
 * 位置控制服务器合并节点实现
 * ================================================================================
 */

#include "basic_control_topic/components/position_servers_node.hpp"
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <sstream>

using namespace std::placeholders;

namespace basic_control_topic
{

// 时间参数化速度/加速度缩放
static constexpr double TOTG_VELOCITY_SCALING = 0.5;
static constexpr double TOTG_ACCELERATION_SCALING = 0.5;

// ============================================================================
// 构造函数
// ============================================================================
PositionServersNode::PositionServersNode(const SharedResources::Ptr& shared_resources,
                                         const rclcpp::NodeOptions& options)
    : Node("position_servers_node", options)
    , shared_resources_(shared_resources)
    , js_monitor_(this)
{
    // ========== 关节位置控制 Action Server ==========
    joint_position_server_ = rclcpp_action::create_server<JointPosition>(
        this,
        "/algorithm/grasp_planning/move/joint_position",
        std::bind(&PositionServersNode::handleJointPositionGoal, this, _1, _2),
        std::bind(&PositionServersNode::handleJointPositionCancel, this, _1),
        std::bind(&PositionServersNode::handleJointPositionAccepted, this, _1));

    // ========== TCP位置控制 Action Server（左右臂）==========
    left_tcp_position_server_ = rclcpp_action::create_server<TcpPosition>(
        this,
        "/algorithm/grasp_planning/move/left_arm/tcp_position",
        std::bind(&PositionServersNode::handleTcpPositionGoal, this, _1, _2, "left"),
        std::bind(&PositionServersNode::handleTcpPositionCancel, this, _1),
        std::bind(&PositionServersNode::handleTcpPositionAccepted, this, _1));

    right_tcp_position_server_ = rclcpp_action::create_server<TcpPosition>(
        this,
        "/algorithm/grasp_planning/move/right_arm/tcp_position",
        std::bind(&PositionServersNode::handleTcpPositionGoal, this, _1, _2, "right"),
        std::bind(&PositionServersNode::handleTcpPositionCancel, this, _1),
        std::bind(&PositionServersNode::handleTcpPositionAccepted, this, _1));

    // ========== 关节增量控制 Action Server ==========
    joint_position_delta_server_ = rclcpp_action::create_server<JointPositionDelta>(
        this,
        "/algorithm/grasp_planning/move/joint_position_delta",
        std::bind(&PositionServersNode::handleJointPositionDeltaGoal, this, _1, _2),
        std::bind(&PositionServersNode::handleJointPositionDeltaCancel, this, _1),
        std::bind(&PositionServersNode::handleJointPositionDeltaAccepted, this, _1));

    // ========== TCP增量控制 Action Server ==========
    // 订阅 TCP 位姿（用于增量控制）
    left_tcp_pose_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpPose>(
        "/algorithm/grasp_planning/left_arm/tcp_pose",
        10,
        std::bind(&PositionServersNode::leftTcpPoseCallback, this, _1));

    right_tcp_pose_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpPose>(
        "/algorithm/grasp_planning/right_arm/tcp_pose",
        10,
        std::bind(&PositionServersNode::rightTcpPoseCallback, this, _1));

    left_tcp_position_delta_server_ = rclcpp_action::create_server<TcpPositionDelta>(
        this,
        "/algorithm/grasp_planning/move/left_arm/tcp_position_delta",
        std::bind(&PositionServersNode::handleTcpPositionDeltaGoal, this, _1, _2, "left"),
        std::bind(&PositionServersNode::handleTcpPositionDeltaCancel, this, _1),
        std::bind(&PositionServersNode::handleTcpPositionDeltaAccepted, this, _1));

    right_tcp_position_delta_server_ = rclcpp_action::create_server<TcpPositionDelta>(
        this,
        "/algorithm/grasp_planning/move/right_arm/tcp_position_delta",
        std::bind(&PositionServersNode::handleTcpPositionDeltaGoal, this, _1, _2, "right"),
        std::bind(&PositionServersNode::handleTcpPositionDeltaCancel, this, _1),
        std::bind(&PositionServersNode::handleTcpPositionDeltaAccepted, this, _1));

    // ========== 初始化 MoveIt 持久化资源 ==========
    initializeMoveIt();

    RCLCPP_INFO(this->get_logger(), "Position Servers Node initialized");
    RCLCPP_INFO(this->get_logger(), "  - Joint Position: /algorithm/grasp_planning/move/joint_position");
    RCLCPP_INFO(this->get_logger(), "  - TCP Position (left): /algorithm/grasp_planning/move/left_arm/tcp_position");
    RCLCPP_INFO(this->get_logger(), "  - TCP Position (right): /algorithm/grasp_planning/move/right_arm/tcp_position");
    RCLCPP_INFO(this->get_logger(), "  - Joint Delta: /algorithm/grasp_planning/move/joint_position_delta");
    RCLCPP_INFO(this->get_logger(), "  - TCP Delta (left): /algorithm/grasp_planning/move/left_arm/tcp_position_delta");
    RCLCPP_INFO(this->get_logger(), "  - TCP Delta (right): /algorithm/grasp_planning/move/right_arm/tcp_position_delta");

    // ========== 注册急停回调 ==========
    shared_resources_->registerEmergencyStopCallback([this]() {
        RCLCPP_WARN(this->get_logger(), "Emergency stop triggered in position servers");
    });
}

// ============================================================================
// 析构函数
// ============================================================================
PositionServersNode::~PositionServersNode()
{
    cleanupMoveIt();
}

// ============================================================================
// MoveIt 资源管理
// ============================================================================
void PositionServersNode::initializeMoveIt()
{
    RCLCPP_INFO(this->get_logger(), "Initializing MoveIt resources...");

    // 创建专用的 MoveIt node
    moveit_node_ = rclcpp::Node::make_shared("moveit_persistent_node");

    // 创建独立 executor
    moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    moveit_executor_->add_node(moveit_node_);

    // 启动 executor 线程
    moveit_thread_ = std::make_shared<std::thread>([this]() {
        moveit_executor_->spin();
    });

    // 创建 MoveGroupInterface（这会启动 CurrentStateMonitor 订阅）
    left_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node_, "left_arm_group");
    right_arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        moveit_node_, "right_arm_group");

    // 配置默认参数
    left_arm_group_->setPlanningTime(10.0);
    left_arm_group_->setGoalJointTolerance(0.01);
    left_arm_group_->setGoalPositionTolerance(0.005);

    right_arm_group_->setPlanningTime(10.0);
    right_arm_group_->setGoalJointTolerance(0.01);
    right_arm_group_->setGoalPositionTolerance(0.005);

    RCLCPP_INFO(this->get_logger(), "MoveIt resources initialized (left_arm_group, right_arm_group)");
}

void PositionServersNode::cleanupMoveIt()
{
    RCLCPP_INFO(this->get_logger(), "Cleaning up MoveIt resources...");

    // 先销毁 MoveGroupInterface
    left_arm_group_.reset();
    right_arm_group_.reset();

    // 停止 executor
    if (moveit_executor_) {
        moveit_executor_->cancel();
    }

    // 等待线程结束
    if (moveit_thread_ && moveit_thread_->joinable()) {
        moveit_thread_->join();
    }

    RCLCPP_INFO(this->get_logger(), "MoveIt resources cleaned up");
}

std::shared_ptr<moveit::planning_interface::MoveGroupInterface> PositionServersNode::getMoveGroup(const std::string& arm_type)
{
    std::lock_guard<std::mutex> lock(moveit_mutex_);
    if (arm_type == "left" || arm_type.find("left_") != std::string::npos) {
        return left_arm_group_;
    } else {
        return right_arm_group_;
    }
}

// ============================================================================
// 关节位置控制
// ============================================================================
rclcpp_action::GoalResponse PositionServersNode::handleJointPositionGoal(
    const GoalUUID& uuid,
    std::shared_ptr<const JointPosition::Goal> goal)
{
    (void)uuid;

    // 急停检查：双重验证
    if (shared_resources_->isEmergencyStopActive()) {
        int current_mode = shared_resources_->getRunMode();
        if (current_mode == 7) {
            RCLCPP_WARN(this->get_logger(), "Joint position goal rejected: emergency stop active");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    if (goal->target_state.name.size() != 7)
    {
        RCLCPP_ERROR(this->get_logger(), "目标关节数量错误！需7个关节，当前为 %zu 个",
                     goal->target_state.name.size());
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 检查目标位置是否在软限位内
    auto limits = shared_resources_->getAllJointLimits();
    for (size_t i = 0; i < goal->target_state.name.size() && i < goal->target_state.position.size(); ++i)
    {
        auto lit = limits.find(goal->target_state.name[i]);
        if (lit != limits.end())
        {
            double pos = goal->target_state.position[i];
            if (pos < lit->second.min_pos || pos > lit->second.max_pos)
            {
                RCLCPP_ERROR(this->get_logger(),
                             "Goal REJECTED: [%s] target=%.3f exceeds soft limits [%.3f, %.3f]",
                             goal->target_state.name[i].c_str(), pos,
                             lit->second.min_pos, lit->second.max_pos);
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
    }

    std::string joint_names;
    for (const auto& name : goal->target_state.name)
    {
        joint_names += name + ", ";
    }
    RCLCPP_INFO(this->get_logger(), "收到关节移动请求，关节列表：%s", joint_names.c_str());

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PositionServersNode::handleJointPositionCancel(
    const std::shared_ptr<GoalHandleJointPosition> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "收到关节移动取消请求");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PositionServersNode::handleJointPositionAccepted(
    const std::shared_ptr<GoalHandleJointPosition> goal_handle)
{
    std::thread{std::bind(&PositionServersNode::executeJointPosition, this, goal_handle)}.detach();
}

void PositionServersNode::executeJointPosition(
    const std::shared_ptr<GoalHandleJointPosition> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<JointPosition::Feedback>();
    auto result = std::make_shared<JointPosition::Result>();

    // 急停恢复后刷新位置
    if (shared_resources_->needRefreshStartState()) {
        RCLCPP_INFO(this->get_logger(), "Refreshing start state after emergency stop recovery");
        if (shared_resources_->waitForFreshJointState(5.0)) {
            RCLCPP_INFO(this->get_logger(), "Joint state refreshed");
        }
        shared_resources_->clearRefreshFlag();
    }

    // 确定手臂类型并获取对应的 MoveGroupInterface
    std::string arm_type;
    if (goal->target_state.name[0].find("left_") != std::string::npos)
    {
        arm_type = "left";
    }
    else if (goal->target_state.name[0].find("right_") != std::string::npos)
    {
        arm_type = "right";
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "关节名称无 'left_' 或 'right_' 前缀");
        result->error_code = 4;
        goal_handle->abort(result);
        return;
    }

    // 使用持久化的 MoveGroupInterface
    auto move_group = getMoveGroup(arm_type);
    if (!move_group)
    {
        RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized for %s", arm_type.c_str());
        result->error_code = 4;
        goal_handle->abort(result);
        return;
    }

    std::string move_group_name = (arm_type == "left") ? "left_arm_group" : "right_arm_group";
    move_group->setPlanningTime(30.0);
    move_group->setGoalJointTolerance(0.01);
    move_group->setGoalPositionTolerance(0.005);

    std::vector<double> target_positions(goal->target_state.position.begin(),
                                          goal->target_state.position.end());

    // 添加关节软限位路径约束
    moveit_msgs::msg::Constraints joint_constraints;
    auto limits = shared_resources_->getAllJointLimits();
    for (size_t i = 0; i < goal->target_state.name.size() && i < target_positions.size(); ++i)
    {
        auto lit = limits.find(goal->target_state.name[i]);
        if (lit != limits.end())
        {
            double center = (lit->second.min_pos + lit->second.max_pos) / 2.0;
            moveit_msgs::msg::JointConstraint jc;
            jc.joint_name = goal->target_state.name[i];
            jc.position = center;
            jc.tolerance_above = lit->second.max_pos - center;
            jc.tolerance_below = center - lit->second.min_pos;
            jc.weight = 1.0;
            joint_constraints.joint_constraints.push_back(jc);
        }
    }
    move_group->setPathConstraints(joint_constraints);
    move_group->setJointValueTarget(goal->target_state.name, target_positions);

    RCLCPP_INFO(this->get_logger(), "已设置 %s 目标关节角，开始规划路径", move_group_name.c_str());

    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success = false;

    // 尝试 PTP 规划
    move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group->setPlannerId("PTP");
    plan_success = (move_group->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // PTP 失败则尝试 OMPL
    if (!plan_success)
    {
        RCLCPP_WARN(this->get_logger(), "PTP 规划失败，尝试 RRTConnect...");
        move_group->setPlanningPipelineId("ompl");
        move_group->setPlannerId("RRTConnect");
        plan_success = (move_group->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    if (!plan_success)
    {
        RCLCPP_ERROR(this->get_logger(), "%s 运动规划失败！", move_group_name.c_str());
        result->timestamp = this->get_clock()->now();
        result->error_code = 2;
        goal_handle->abort(result);
        return;
    }

    RCLCPP_INFO(this->get_logger(), "%s 规划成功！耗时：%.2f 秒",
                move_group_name.c_str(), motion_plan.planning_time_);

    // 时间参数化
    robot_trajectory::RobotTrajectory robot_traj(move_group->getRobotModel(), move_group_name);
    robot_traj.setRobotTrajectoryMsg(*move_group->getCurrentState(), motion_plan.trajectory_);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    double vel_scale = (goal->velocity == 0) ? TOTG_VELOCITY_SCALING
                                              : goal->velocity / 100.0 * 0.5;
    bool time_param_success = totg.computeTimeStamps(robot_traj, vel_scale, vel_scale);

    if (time_param_success)
    {
        robot_traj.getRobotTrajectoryMsg(motion_plan.trajectory_);
        RCLCPP_INFO(this->get_logger(), "时间参数化成功，轨迹点数: %zu",
                    motion_plan.trajectory_.joint_trajectory.points.size());
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "时间参数化失败！");
        result->timestamp = this->get_clock()->now();
        result->error_code = 5;
        goal_handle->abort(result);
        return;
    }

    // 执行运动
    bool execute_success = js_monitor_.executeWithTimeout(*move_group, motion_plan, 3.0);

    if (!execute_success)
    {
        RCLCPP_ERROR(this->get_logger(), "%s 运动执行失败", move_group_name.c_str());
        result->timestamp = this->get_clock()->now();
        result->error_code = 3;
        goal_handle->abort(result);
        return;
    }

    // 验证执行结果
    if (!js_monitor_.verifyExecution(goal->target_state.name, target_positions))
    {
        RCLCPP_ERROR(this->get_logger(), "%s 执行验证失败", move_group_name.c_str());
        result->timestamp = this->get_clock()->now();
        result->error_code = 3;
        goal_handle->abort(result);
        return;
    }

    feedback->timestamp = this->get_clock()->now();
    feedback->process_status = 100;
    goal_handle->publish_feedback(feedback);

    result->timestamp = this->get_clock()->now();
    result->planning_time.sec = static_cast<int>(motion_plan.planning_time_);
    result->planning_time.nanosec = static_cast<int>(
        (motion_plan.planning_time_ - result->planning_time.sec) * 1e9);
    result->error_code = 0;
    goal_handle->succeed(result);

    RCLCPP_INFO(this->get_logger(), "%s 已成功移动到目标位置！", move_group_name.c_str());
}

// ============================================================================
// TCP位置控制
// ============================================================================
rclcpp_action::GoalResponse PositionServersNode::handleTcpPositionGoal(
    const GoalUUID& uuid,
    std::shared_ptr<const TcpPosition::Goal> goal,
    const std::string& arm_type)
{
    // 急停检查：双重验证
    if (shared_resources_->isEmergencyStopActive()) {
        int current_mode = shared_resources_->getRunMode();
        if (current_mode == 7) {
            RCLCPP_WARN(this->get_logger(), "%s TCP position goal rejected: emergency stop active", arm_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // 四元数验证
    double quat_norm = goal->pose.pose.orientation.x * goal->pose.pose.orientation.x +
                       goal->pose.pose.orientation.y * goal->pose.pose.orientation.y +
                       goal->pose.pose.orientation.z * goal->pose.pose.orientation.z +
                       goal->pose.pose.orientation.w * goal->pose.pose.orientation.w;

    if (std::abs(quat_norm - 1.0) > 0.01)
    {
        RCLCPP_ERROR(this->get_logger(), "目标姿态四元数未归一化（模长=%.3f），拒绝请求", quat_norm);
        return rclcpp_action::GoalResponse::REJECT;
    }

    {
        std::lock_guard<std::mutex> lock(tcp_position_map_mutex_);
        tcp_position_goal_arm_map_[uuid] = arm_type;
    }

    RCLCPP_INFO(this->get_logger(), "收到%s臂末端移动请求，旋转样式: %d",
                arm_type.c_str(), goal->rotation_style);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PositionServersNode::handleTcpPositionCancel(
    const std::shared_ptr<GoalHandleTcpPosition> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "收到末端移动取消请求");
    std::lock_guard<std::mutex> lock(tcp_position_map_mutex_);
    tcp_position_goal_arm_map_.erase(goal_handle->get_goal_id());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PositionServersNode::handleTcpPositionAccepted(
    const std::shared_ptr<GoalHandleTcpPosition> goal_handle)
{
    std::thread{std::bind(&PositionServersNode::executeTcpPosition, this, goal_handle)}.detach();
}

void PositionServersNode::executeTcpPosition(
    const std::shared_ptr<GoalHandleTcpPosition> goal_handle)
{
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TcpPosition::Feedback>();
    auto result = std::make_shared<TcpPosition::Result>();
    result->error_code = 0;

    // 急停恢复后刷新位置
    if (shared_resources_->needRefreshStartState()) {
        RCLCPP_INFO(this->get_logger(), "Refreshing start state after emergency stop recovery");
        if (shared_resources_->waitForFreshJointState(5.0)) {
            RCLCPP_INFO(this->get_logger(), "Joint state refreshed");
        }
        shared_resources_->clearRefreshFlag();
    }

    // 查询手臂类型
    GoalUUID goal_id = goal_handle->get_goal_id();
    std::string arm_type;
    {
        std::lock_guard<std::mutex> lock(tcp_position_map_mutex_);
        auto it = tcp_position_goal_arm_map_.find(goal_id);
        if (it == tcp_position_goal_arm_map_.end())
        {
            RCLCPP_ERROR(this->get_logger(), "未找到目标ID对应的手臂类型");
            result->error_code = 4;
            goal_handle->abort(result);
            return;
        }
        arm_type = it->second;
    }

    std::string move_group_name = (arm_type == "left") ? "left_arm_group" : "right_arm_group";

    RCLCPP_INFO(this->get_logger(), "开始处理%s臂末端移动请求", arm_type.c_str());

    // 使用持久化的 MoveGroupInterface
    auto move_group = getMoveGroup(arm_type);
    if (!move_group)
    {
        RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized for %s", arm_type.c_str());
        result->error_code = 4;
        goal_handle->abort(result);
        std::lock_guard<std::mutex> lock(tcp_position_map_mutex_);
        tcp_position_goal_arm_map_.erase(goal_id);
        return;
    }

    move_group->setPlanningTime(10.0);
    move_group->setGoalPositionTolerance(0.005);
    move_group->setGoalOrientationTolerance(0.01);

    // 添加关节软限位路径约束
    {
        std::vector<std::string> group_joints = move_group->getJoints();
        moveit_msgs::msg::Constraints joint_constraints;
        auto limits = shared_resources_->getAllJointLimits();
        for (const auto& jname : group_joints)
        {
            auto lit = limits.find(jname);
            if (lit != limits.end())
            {
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
        move_group->setPathConstraints(joint_constraints);
    }

    move_group->setPoseTarget(goal->pose);

    // 尝试 Cartesian 直线规划
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success = false;

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group->getCurrentPose().pose);
    waypoints.push_back(goal->pose.pose);

    double fraction = move_group->computeCartesianPath(
        waypoints, 0.01, 0.0, motion_plan.trajectory_);

    if (fraction >= 0.99)
    {
        plan_success = true;
        motion_plan.planning_time_ = 0.1;
        RCLCPP_INFO(this->get_logger(), "%s臂 Cartesian 直线规划成功，完成度: %.1f%%",
                    arm_type.c_str(), fraction * 100);
    }
    else
    {
        RCLCPP_WARN(this->get_logger(), "%s臂 Cartesian 不完整，尝试 OMPL", arm_type.c_str());
        plan_success = (move_group->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    }

    if (!plan_success)
    {
        RCLCPP_ERROR(this->get_logger(), "%s臂轨迹规划失败", arm_type.c_str());
        result->error_code = 2;
        goal_handle->abort(result);
        std::lock_guard<std::mutex> lock(tcp_position_map_mutex_);
        tcp_position_goal_arm_map_.erase(goal_id);
        return;
    }

    // 时间参数化
    robot_trajectory::RobotTrajectory robot_traj(move_group->getRobotModel(), move_group_name);
    robot_traj.setRobotTrajectoryMsg(*move_group->getCurrentState(), motion_plan.trajectory_);

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    double velocity_scaling = (goal->velocity == 0) ? 0.01 : goal->velocity / 100.0 * 0.1;
    double acceleration_scaling = 0.01;
    totg.computeTimeStamps(robot_traj, velocity_scaling, acceleration_scaling);
    robot_traj.getRobotTrajectoryMsg(motion_plan.trajectory_);

    // 执行
    std::vector<std::string> joint_names = move_group->getJoints();
    bool execute_success = js_monitor_.executeWithTimeout(*move_group, motion_plan, 3.0);

    if (!execute_success)
    {
        RCLCPP_ERROR(this->get_logger(), "%s臂轨迹执行失败", arm_type.c_str());
        result->error_code = 3;
        goal_handle->abort(result);
    }
    else
    {
        auto final_state = move_group->getCurrentState();
        std::vector<double> final_positions;
        final_state->copyJointGroupPositions(move_group_name, final_positions);

        if (!js_monitor_.verifyExecution(joint_names, final_positions))
        {
            RCLCPP_ERROR(this->get_logger(), "%s臂执行验证失败", arm_type.c_str());
            result->error_code = 3;
            goal_handle->abort(result);
        }
        else
        {
            feedback->timestamp = this->get_clock()->now();
            feedback->process_status = 100;
            goal_handle->publish_feedback(feedback);

            result->timestamp = this->get_clock()->now();
            result->planning_time.sec = static_cast<int>(motion_plan.planning_time_);
            result->planning_time.nanosec = static_cast<int>(
                (motion_plan.planning_time_ - result->planning_time.sec) * 1e9);
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "%s臂末端已成功到达目标位置", arm_type.c_str());
        }
    }

    std::lock_guard<std::mutex> lock(tcp_position_map_mutex_);
    tcp_position_goal_arm_map_.erase(goal_id);
}

// ============================================================================
// 关节增量控制
// ============================================================================
bool PositionServersNode::identifyJointGroup(
    const std::vector<std::string>& joint_names,
    std::string& joint_group_name)
{
    if (joint_names.empty()) return false;

    const std::string& first_joint = joint_names[0];
    if (first_joint.find("left_") != std::string::npos)
    {
        joint_group_name = "left_arm_group";
    }
    else if (first_joint.find("right_") != std::string::npos)
    {
        joint_group_name = "right_arm_group";
    }
    else
    {
        return false;
    }

    for (const auto& joint_name : joint_names)
    {
        bool is_left = (joint_name.find("left_") != std::string::npos);
        bool is_right = (joint_name.find("right_") != std::string::npos);
        bool match = (joint_group_name == "left_arm_group" && is_left) ||
                     (joint_group_name == "right_arm_group" && is_right);
        if (!match) return false;
    }
    return true;
}

rclcpp_action::GoalResponse PositionServersNode::handleJointPositionDeltaGoal(
    const GoalUUID& uuid,
    std::shared_ptr<const JointPositionDelta::Goal> goal)
{
    (void)uuid;

    // 急停检查：双重验证
    if (shared_resources_->isEmergencyStopActive()) {
        int current_mode = shared_resources_->getRunMode();
        if (current_mode == 7) {
            RCLCPP_WARN(this->get_logger(), "Joint position delta goal rejected: emergency stop active");
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    if (goal->name.size() != goal->position_delta.size())
    {
        RCLCPP_ERROR(this->get_logger(), "关节名数组长度与增量数组长度不匹配！");
        return rclcpp_action::GoalResponse::REJECT;
    }

    std::string joint_group_name;
    if (!identifyJointGroup(goal->name, joint_group_name))
    {
        RCLCPP_ERROR(this->get_logger(), "无法识别关节组");
        return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(this->get_logger(), "收到有效请求：控制关节组[%s]", joint_group_name.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PositionServersNode::handleJointPositionDeltaCancel(
    const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PositionServersNode::handleJointPositionDeltaAccepted(
    const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle)
{
    std::thread{std::bind(&PositionServersNode::executeJointPositionDelta, this, goal_handle)}.detach();
}

void PositionServersNode::executeJointPositionDelta(
    const std::shared_ptr<GoalHandleJointPositionDelta> goal_handle)
{
    std::lock_guard<std::mutex> exec_lock(joint_delta_execute_mutex_);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<JointPositionDelta::Feedback>();
    auto result = std::make_shared<JointPositionDelta::Result>();
    result->timestamp = this->get_clock()->now();

    // 急停恢复后刷新位置
    if (shared_resources_->needRefreshStartState()) {
        RCLCPP_INFO(this->get_logger(), "Refreshing start state after emergency stop recovery");
        if (shared_resources_->waitForFreshJointState(5.0)) {
            RCLCPP_INFO(this->get_logger(), "Joint state refreshed");
        }
        shared_resources_->clearRefreshFlag();
    }

    std::string joint_group_name;
    identifyJointGroup(goal->name, joint_group_name);

    // 确定手臂类型
    std::string arm_type;
    if (joint_group_name == "left_arm_group") {
        arm_type = "left";
    } else {
        arm_type = "right";
    }

    // 使用持久化的 MoveGroupInterface
    auto move_group = getMoveGroup(arm_type);
    if (!move_group)
    {
        RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized for %s", arm_type.c_str());
        result->error_code = 4;
        goal_handle->abort(result);
        return;
    }

    move_group->setPlanningTime(10.0);
    move_group->setGoalJointTolerance(0.01);
    double vel_scale = (goal->velocity == 0) ? 0.3 : goal->velocity / 100.0 * 0.5;
    move_group->setMaxVelocityScalingFactor(vel_scale);
    move_group->setMaxAccelerationScalingFactor(vel_scale);
    move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group->setPlannerId("PTP");

    std::vector<std::string> all_group_joints = move_group->getJoints();
    if (all_group_joints.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "关节组无关节配置");
        result->error_code = 4;
        goal_handle->abort(result);
        return;
    }

    // 计算目标位置
    std::unordered_map<std::string, double> joint_pos_snapshot = shared_resources_->getAllJointPositions();
    std::unordered_map<std::string, double> request_joint_delta_map;
    for (size_t i = 0; i < goal->name.size(); ++i)
    {
        request_joint_delta_map[goal->name[i]] = static_cast<double>(goal->position_delta[i]);
    }

    std::vector<double> target_positions;
    for (const auto& joint : all_group_joints)
    {
        double current_pos = joint_pos_snapshot.count(joint) ? joint_pos_snapshot[joint] : 0.0;
        auto delta_it = request_joint_delta_map.find(joint);
        if (delta_it != request_joint_delta_map.end())
        {
            target_positions.push_back(current_pos + delta_it->second);
        }
        else
        {
            target_positions.push_back(current_pos);
        }
    }

    // 验证目标位置在软限位内
    auto limits = shared_resources_->getAllJointLimits();
    for (size_t i = 0; i < all_group_joints.size() && i < target_positions.size(); ++i)
    {
        auto lit = limits.find(all_group_joints[i]);
        if (lit != limits.end())
        {
            double pos = target_positions[i];
            if (pos < lit->second.min_pos || pos > lit->second.max_pos)
            {
                RCLCPP_ERROR(this->get_logger(), "Target REJECTED [%s] pos=%.3f",
                            all_group_joints[i].c_str(), pos);
                result->error_code = 5;
                goal_handle->abort(result);
                return;
            }
        }
    }

    // 添加关节约束
    moveit_msgs::msg::Constraints joint_constraints;
    for (size_t i = 0; i < all_group_joints.size(); ++i)
    {
        auto lit = limits.find(all_group_joints[i]);
        if (lit != limits.end())
        {
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
    move_group->setPathConstraints(joint_constraints);
    move_group->setJointValueTarget(all_group_joints, target_positions);

    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success = (move_group->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_success)
    {
        RCLCPP_ERROR(this->get_logger(), "规划失败");
        result->error_code = 2;
        goal_handle->abort(result);
        return;
    }

    // 执行
    bool execute_success = js_monitor_.executeWithTimeout(*move_group, motion_plan, 3.0);

    if (!execute_success)
    {
        RCLCPP_ERROR(this->get_logger(), "执行失败");
        result->error_code = 3;
        goal_handle->abort(result);
    }
    else if (!js_monitor_.verifyExecution(all_group_joints, target_positions))
    {
        RCLCPP_ERROR(this->get_logger(), "验证失败");
        result->error_code = 3;
        goal_handle->abort(result);
    }
    else
    {
        result->error_code = 0;
        result->planning_time.sec = static_cast<int32_t>(motion_plan.planning_time_);
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "关节增量移动成功");
    }
}

// ============================================================================
// TCP增量控制
// ============================================================================
void PositionServersNode::leftTcpPoseCallback(const planning_sdk_msgs::msg::TcpPose::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(left_pose_mutex_);
    latest_left_tcp_pose_ = *msg;
    left_pose_received_ = (msg->error_code == 0);
}

void PositionServersNode::rightTcpPoseCallback(const planning_sdk_msgs::msg::TcpPose::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(right_pose_mutex_);
    latest_right_tcp_pose_ = *msg;
    right_pose_received_ = (msg->error_code == 0);
}

bool PositionServersNode::getLatestTcpPose(
    const std::string& arm_type,
    geometry_msgs::msg::PoseStamped& current_pose)
{
    int retry_count = 0;
    const int max_retry = 6;
    const std::chrono::milliseconds retry_delay = std::chrono::milliseconds(500);

    while (retry_count < max_retry && rclcpp::ok())
    {
        if (arm_type == "left")
        {
            std::lock_guard<std::mutex> lock(left_pose_mutex_);
            if (left_pose_received_)
            {
                current_pose.header.frame_id = "base_link";
                current_pose.header.stamp = latest_left_tcp_pose_.timestamp;
                current_pose.pose = latest_left_tcp_pose_.current_pose;
                return true;
            }
        }
        else if (arm_type == "right")
        {
            std::lock_guard<std::mutex> lock(right_pose_mutex_);
            if (right_pose_received_)
            {
                current_pose.header.frame_id = "base_link";
                current_pose.header.stamp = latest_right_tcp_pose_.timestamp;
                current_pose.pose = latest_right_tcp_pose_.current_pose;
                return true;
            }
        }

        retry_count++;
        std::this_thread::sleep_for(retry_delay);
    }

    RCLCPP_ERROR(this->get_logger(), "[%s臂] 超时未收到有效位姿", arm_type.c_str());
    return false;
}

rclcpp_action::GoalResponse PositionServersNode::handleTcpPositionDeltaGoal(
    const GoalUUID& uuid,
    std::shared_ptr<const TcpPositionDelta::Goal> goal,
    const std::string& arm_type)
{
    // 急停检查：双重验证
    if (shared_resources_->isEmergencyStopActive()) {
        int current_mode = shared_resources_->getRunMode();
        if (current_mode == 7) {
            RCLCPP_WARN(this->get_logger(), "%s TCP position delta goal rejected: emergency stop active", arm_type.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    double quat_norm = std::sqrt(
        goal->pose_delta.pose.orientation.x * goal->pose_delta.pose.orientation.x +
        goal->pose_delta.pose.orientation.y * goal->pose_delta.pose.orientation.y +
        goal->pose_delta.pose.orientation.z * goal->pose_delta.pose.orientation.z +
        goal->pose_delta.pose.orientation.w * goal->pose_delta.pose.orientation.w);

    if (std::abs(quat_norm - 1.0) > 0.01)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s臂] 四元数未归一化", arm_type.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    {
        std::lock_guard<std::mutex> lock(tcp_delta_map_mutex_);
        tcp_delta_goal_arm_map_[uuid] = arm_type;
    }

    RCLCPP_INFO(this->get_logger(), "[%s臂] 收到末端偏移请求", arm_type.c_str());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PositionServersNode::handleTcpPositionDeltaCancel(
    const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "收到取消请求");
    std::lock_guard<std::mutex> lock(tcp_delta_map_mutex_);
    tcp_delta_goal_arm_map_.erase(goal_handle->get_goal_id());
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PositionServersNode::handleTcpPositionDeltaAccepted(
    const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle)
{
    std::thread{std::bind(&PositionServersNode::executeTcpPositionDelta, this, goal_handle)}.detach();
}

void PositionServersNode::executeTcpPositionDelta(
    const std::shared_ptr<GoalHandleTcpPositionDelta> goal_handle)
{
    std::lock_guard<std::mutex> exec_lock(tcp_delta_execute_mutex_);

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TcpPositionDelta::Feedback>();
    auto result = std::make_shared<TcpPositionDelta::Result>();
    result->error_code = 0;

    // 急停恢复后刷新位置
    if (shared_resources_->needRefreshStartState()) {
        RCLCPP_INFO(this->get_logger(), "Refreshing start state after emergency stop recovery");
        if (shared_resources_->waitForFreshJointState(5.0)) {
            RCLCPP_INFO(this->get_logger(), "Joint state refreshed");
        }
        shared_resources_->clearRefreshFlag();
    }

    GoalUUID goal_id = goal_handle->get_goal_id();
    std::string arm_type;
    {
        std::lock_guard<std::mutex> lock(tcp_delta_map_mutex_);
        auto it = tcp_delta_goal_arm_map_.find(goal_id);
        if (it == tcp_delta_goal_arm_map_.end())
        {
            result->error_code = 4;
            goal_handle->abort(result);
            return;
        }
        arm_type = it->second;
    }

    std::string move_group_name = (arm_type == "left") ? "left_arm_group" : "right_arm_group";
    std::string end_effector_link = (arm_type == "left") ? left_end_effector_link_ : right_end_effector_link_;

    geometry_msgs::msg::PoseStamped current_pose;
    if (!getLatestTcpPose(arm_type, current_pose))
    {
        result->error_code = 4;
        goal_handle->abort(result);
        std::lock_guard<std::mutex> lock(tcp_delta_map_mutex_);
        tcp_delta_goal_arm_map_.erase(goal_id);
        return;
    }

    // 计算目标位姿
    geometry_msgs::msg::PoseStamped target_pose = current_pose;
    target_pose.pose.position.x += goal->pose_delta.pose.position.x;
    target_pose.pose.position.y += goal->pose_delta.pose.position.y;
    target_pose.pose.position.z += goal->pose_delta.pose.position.z;

    tf2::Quaternion current_quat, delta_quat, target_quat;
    tf2::fromMsg(current_pose.pose.orientation, current_quat);
    tf2::fromMsg(goal->pose_delta.pose.orientation, delta_quat);
    target_quat = delta_quat * current_quat;
    target_quat.normalize();
    target_pose.pose.orientation = tf2::toMsg(target_quat);

    // 无偏移请求直接返回成功
    double pos_offset_mag = std::sqrt(
        std::pow(goal->pose_delta.pose.position.x, 2) +
        std::pow(goal->pose_delta.pose.position.y, 2) +
        std::pow(goal->pose_delta.pose.position.z, 2));

    if (pos_offset_mag < 1e-3)
    {
        RCLCPP_INFO(this->get_logger(), "[%s臂] 无偏移请求，直接返回成功", arm_type.c_str());
        feedback->timestamp = this->get_clock()->now();
        feedback->process_status = 100;
        goal_handle->publish_feedback(feedback);
        result->timestamp = this->get_clock()->now();
        goal_handle->succeed(result);
        std::lock_guard<std::mutex> lock(tcp_delta_map_mutex_);
        tcp_delta_goal_arm_map_.erase(goal_id);
        return;
    }

    // 使用持久化的 MoveGroupInterface
    auto move_group = getMoveGroup(arm_type);
    if (!move_group)
    {
        RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface not initialized for %s", arm_type.c_str());
        result->error_code = 4;
        goal_handle->abort(result);
        std::lock_guard<std::mutex> lock(tcp_delta_map_mutex_);
        tcp_delta_goal_arm_map_.erase(goal_id);
        return;
    }

    move_group->setEndEffectorLink(end_effector_link);
    move_group->setPlanningTime(15.0);
    move_group->setGoalPositionTolerance(0.005);
    move_group->setGoalOrientationTolerance(0.01);
    move_group->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group->setPlannerId("PTP");

    // 添加关节约束
    {
        std::vector<std::string> group_joints = move_group->getJoints();
        moveit_msgs::msg::Constraints path_constraints;
        auto limits = shared_resources_->getAllJointLimits();
        for (const auto& jname : group_joints)
        {
            auto lit = limits.find(jname);
            if (lit != limits.end())
            {
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
        move_group->setPathConstraints(path_constraints);
    }

    move_group->setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan motion_plan;
    bool plan_success = (move_group->plan(motion_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!plan_success)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s臂] 规划失败", arm_type.c_str());
        result->error_code = 2;
        goal_handle->abort(result);
        std::lock_guard<std::mutex> lock(tcp_delta_map_mutex_);
        tcp_delta_goal_arm_map_.erase(goal_id);
        return;
    }

    std::vector<std::string> joint_names = move_group->getJoints();
    bool execute_success = js_monitor_.executeWithTimeout(*move_group, motion_plan, 3.0);

    if (!execute_success)
    {
        RCLCPP_ERROR(this->get_logger(), "[%s臂] 执行失败", arm_type.c_str());
        result->error_code = 3;
        goal_handle->abort(result);
    }
    else
    {
        auto final_state = move_group->getCurrentState();
        std::vector<double> final_positions;
        final_state->copyJointGroupPositions(move_group_name, final_positions);

        if (!js_monitor_.verifyExecution(joint_names, final_positions))
        {
            RCLCPP_ERROR(this->get_logger(), "[%s臂] 验证失败", arm_type.c_str());
            result->error_code = 3;
            goal_handle->abort(result);
        }
        else
        {
            feedback->timestamp = this->get_clock()->now();
            feedback->process_status = 100;
            goal_handle->publish_feedback(feedback);
            result->timestamp = this->get_clock()->now();
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "[%s臂] 末端偏移执行成功", arm_type.c_str());
        }
    }

    std::lock_guard<std::mutex> lock(tcp_delta_map_mutex_);
    tcp_delta_goal_arm_map_.erase(goal_id);
}

}  // namespace basic_control_topic
