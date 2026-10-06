/**
 * ================================================================================
 * 共享资源管理类 (Shared Resources)
 * ================================================================================
 *
 * 用于在单进程多节点架构中共享常用数据和订阅，避免重复订阅和内存浪费。
 *
 * 共享内容:
 *   - 关节限位缓存 (joint_pos_limits_)
 *   - 关节状态缓存 (current_joint_positions_)
 *   - 对应的互斥锁保护线程安全
 *
 * 使用方式:
 *   1. 创建 SharedResources 实例
 *   2. 调用 initialize() 初始化共享订阅
 *   3. 各组件通过 getJointLimits() / getJointPosition() 访问数据
 *
 * ================================================================================
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <planning_sdk_msgs/msg/joint_limits_array.hpp>
#include <planning_sdk_msgs/msg/joint_limit.hpp>
#include <planning_sdk_msgs/action/joint_position.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <cerebellum_sdk_msg/msg/motor_state.hpp>
#include <cerebellum_sdk_msg/srv/motor_mode.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <unordered_map>
#include <mutex>
#include <memory>
#include <string>
#include <atomic>
#include <thread>
#include <chrono>
#include <functional>
#include <vector>

namespace basic_control_topic
{

/**
 * 关节位置限位数据结构
 */
struct PosLimit
{
    double min_pos;
    double max_pos;
    double velocity_limit;
    double effort_limit;
};

/**
 * 共享资源管理类
 *
 * 管理多个组件共享的数据和订阅，确保线程安全访问。
 */
class SharedResources
{
public:
    using Ptr = std::shared_ptr<SharedResources>;
    using ConstPtr = std::shared_ptr<const SharedResources>;
    using JointPosition = planning_sdk_msgs::action::JointPosition;

    SharedResources() = default;
    ~SharedResources() = default;

    /**
     * 初始化共享订阅
     *
     * @param node 用于创建订阅的 ROS 节点
     * @param monitor_only 仅监控急停/三停状态，不执行控制动作（切位控、发初始姿态等）
     */
    void initialize(rclcpp::Node::SharedPtr node, bool monitor_only = false)
    {
        node_ = node;
        monitor_only_ = monitor_only;

        // 订阅动态关节限位 (transient_local QoS 匹配 server 端)
        auto limits_qos = rclcpp::QoS(1).transient_local();
        joint_limits_sub_ = node_->create_subscription<planning_sdk_msgs::msg::JointLimitsArray>(
            "/algorithm/joint_limits/current",
            limits_qos,
            std::bind(&SharedResources::jointLimitsCallback, this, std::placeholders::_1));

        // 订阅关节状态
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/cerebellum_sdk/arm/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&SharedResources::jointStateCallback, this, std::placeholders::_1));

        // 订阅电机状态（用于急停检测）
        motor_states_sub_ = node_->create_subscription<cerebellum_sdk_msg::msg::MotorState>(
            "/cerebellum_sdk/arm/motor_states",
            10,
            std::bind(&SharedResources::motorStatesCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "SharedResources: motor_states subscription created (%s mode)",
            monitor_only_ ? "monitor-only" : "full control");

        if (!monitor_only_) {
            // 完整模式：创建控制相关的 client（仅 basic_control_node 需要）
            motor_mode_client_ = node_->create_client<cerebellum_sdk_msg::srv::MotorMode>(
                "/cerebellum_sdk/arm/joint_mode");

            switch_controller_cli_ = node_->create_client<controller_manager_msgs::srv::SwitchController>(
                "/controller_manager/switch_controller");

            joint_position_client_ = rclcpp_action::create_client<JointPosition>(
                node_, "/algorithm/grasp_planning/move/joint_position");

            joint_commands_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_commands", 10);

            left_arm_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/left_arm_group_controller/joint_trajectory", rclcpp::QoS(10));
            right_arm_traj_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
                "/right_arm_group_controller/joint_trajectory", rclcpp::QoS(10));

            // 急停恢复完成通知（通知 sim_interface_switch 放行）
            estop_recovery_done_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
                "/algorithm/internal/estop_recovery_done", rclcpp::QoS(1).transient_local());
        }

        // 声明三停状态反转参数（避免重复声明）
        if (!node_->has_parameter("invert_three_stop")) {
            node_->declare_parameter<bool>("invert_three_stop", true);
        }
        invert_three_stop_ = node_->get_parameter("invert_three_stop").as_bool();

        // 订阅小脑 SDK 三停状态
        cerebellum_status_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            "/cerebellum_sdk/three_stop/status", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                bool allow = invert_three_stop_ ? !msg->data : msg->data;
                cerebellum_allow_execute_.store(allow);
                last_cerebellum_msg_time_ = std::chrono::steady_clock::now();
            });

        // monitor_only 模式：订阅恢复完成通知
        if (monitor_only_) {
            auto recovery_qos = rclcpp::QoS(1).transient_local();
            estop_recovery_done_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
                "/algorithm/internal/estop_recovery_done", recovery_qos,
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    if (msg->data && need_refresh_start_state_.load()) {
                        need_refresh_start_state_.store(false);
                        RCLCPP_INFO(node_->get_logger(), "Monitor: recovery complete, cleared refresh flag");
                    }
                });
        }

        RCLCPP_INFO(node_->get_logger(), "SharedResources initialized (%s)",
            monitor_only_ ? "monitor-only" : "full control");
    }

    /**
     * 获取关节限位（线程安全）
     *
     * @param joint_name 关节名称
     * @param limit 输出参数：关节限位数据
     * @return 是否找到该关节的限位数据
     */
    bool getJointLimit(const std::string& joint_name, PosLimit& limit) const
    {
        std::lock_guard<std::mutex> lock(limits_mutex_);
        auto it = joint_pos_limits_.find(joint_name);
        if (it != joint_pos_limits_.end())
        {
            limit = it->second;
            return true;
        }
        return false;
    }

    /**
     * 获取所有关节限位（线程安全）
     *
     * @return 关节限位映射的副本
     */
    std::unordered_map<std::string, PosLimit> getAllJointLimits() const
    {
        std::lock_guard<std::mutex> lock(limits_mutex_);
        return joint_pos_limits_;
    }

    /**
     * 获取关节当前位置（线程安全）
     *
     * @param joint_name 关节名称
     * @param position 输出参数：关节位置
     * @return 是否找到该关节的位置数据
     */
    bool getJointPosition(const std::string& joint_name, double& position) const
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        auto it = current_joint_positions_.find(joint_name);
        if (it != current_joint_positions_.end())
        {
            position = it->second;
            return true;
        }
        return false;
    }

    /**
     * 获取所有关节位置（线程安全）
     *
     * @return 关节位置映射的副本
     */
    std::unordered_map<std::string, double> getAllJointPositions() const
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        return current_joint_positions_;
    }

    /**
     * 检查是否已收到关节状态
     */
    bool hasJointStates() const
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        return !current_joint_positions_.empty();
    }

    /**
     * 检查是否已收到关节限位
     */
    bool hasJointLimits() const
    {
        std::lock_guard<std::mutex> lock(limits_mutex_);
        return !joint_pos_limits_.empty();
    }

    /**
     * 直接访问关节限位映射（需要外部加锁）
     * 用于需要批量操作的场景
     */
    std::unordered_map<std::string, PosLimit>& jointPosLimits()
    {
        return joint_pos_limits_;
    }

    const std::unordered_map<std::string, PosLimit>& jointPosLimits() const
    {
        return joint_pos_limits_;
    }

    std::mutex& limitsMutex() { return limits_mutex_; }

    /**
     * 直接访问关节位置映射（需要外部加锁）
     */
    std::unordered_map<std::string, double>& currentJointPositions()
    {
        return current_joint_positions_;
    }

    const std::unordered_map<std::string, double>& currentJointPositions() const
    {
        return current_joint_positions_;
    }

    std::mutex& jointStateMutex() { return joint_state_mutex_; }

    // ========== 急停相关 ==========

    /**
     * 检查急停是否激活
     */
    bool isEmergencyStopActive() const { return emergency_stop_active_.load(); }

    /**
     * 检查三停是否允许执行（供外部查询）
     * 返回 true 表示允许执行，false 表示暂停
     */
    bool isThreeStopAllowExecute() const {
        int64_t elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_cerebellum_msg_time_).count();
        // 丢帧容忍：超时且上次状态为允许，则继续允许
        if (elapsed_ms > CEREBELLUM_TIMEOUT_MS && cerebellum_allow_execute_.load()) {
            return true;
        }
        return cerebellum_allow_execute_.load() && (elapsed_ms <= CEREBELLUM_TIMEOUT_MS);
    }

    /**
     * 检查是否需要刷新起始状态（急停恢复后）
     */
    bool needRefreshStartState() const { return need_refresh_start_state_.load(); }

    /**
     * 清除刷新标志（各 server 刷新后调用）
     */
    void clearRefreshFlag() { need_refresh_start_state_.store(false); }

    /**
     * 急停时立即停控制器并清空缓冲区（fire-and-forget，不阻塞）
     */
    void haltControllers()
    {
        if (!switch_controller_cli_ || !switch_controller_cli_->service_is_ready()) {
            return;
        }
        auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        req->deactivate_controllers = {"left_arm_group_controller", "right_arm_group_controller"};
        req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
        // fire-and-forget，异步发送，不阻塞
        switch_controller_cli_->async_send_request(req);
        RCLCPP_WARN(node_->get_logger(), "Controllers halt command sent");
    }

    /**
     * 急停时向两个控制器发送停止轨迹（当前位置 + 零速度），覆盖缓冲区中的旧轨迹
     */
    void sendStopTrajectories()
    {
        if (!left_arm_traj_pub_ || !right_arm_traj_pub_) return;

        static const std::vector<std::string> left_names = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint"
        };
        static const std::vector<std::string> right_names = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint", "right_wrist_yaw_joint", "right_wrist_pitch_joint"
        };

        auto build_stop = [this](const std::vector<std::string>& names)
            -> trajectory_msgs::msg::JointTrajectory
        {
            trajectory_msgs::msg::JointTrajectory msg;
            msg.header.stamp = node_->now();
            msg.joint_names = names;
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                for (const auto& name : names) {
                    auto it = current_joint_positions_.find(name);
                    pt.positions.push_back(
                        it != current_joint_positions_.end() ? it->second : 0.0);
                }
            }
            pt.velocities.resize(names.size(), 0.0);
            pt.time_from_start = rclcpp::Duration::from_seconds(0.01);
            msg.points.push_back(pt);
            return msg;
        };

        for (int i = 0; i < 3; ++i) {
            left_arm_traj_pub_->publish(build_stop(left_names));
            right_arm_traj_pub_->publish(build_stop(right_names));
            if (i < 2) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        RCLCPP_WARN(node_->get_logger(), "Stop trajectories sent to both arm controllers");
    }

    /**
     * 注册急停回调（各 action server 注册 move_group.stop() 等）
     * 回调必须是非阻塞的，在 motor_states 回调线程中执行
     */
    void registerEmergencyStopCallback(std::function<void()> cb)
    {
        std::lock_guard<std::mutex> lock(estop_cb_mutex_);
        emergency_stop_callbacks_.push_back(std::move(cb));
    }

    /**
     * 注册队列清空回调（在急停恢复开始时同步调用）
     */
    void registerClearQueueCallback(std::function<void()> cb)
    {
        clear_queue_callback_ = std::move(cb);
    }

    /**
     * 获取当前运行模式
     */
    int getRunMode() const { return current_run_mode_.load(); }

    /**
     * 等待一条新的 /joint_states 消息到达（不使用缓存旧数据）
     */
    bool waitForFreshJointState(double timeout_sec = 5.0)
    {
        fresh_joint_state_received_.store(false);
        auto start = std::chrono::steady_clock::now();
        while (!fresh_joint_state_received_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - start).count() > timeout_sec) {
                RCLCPP_ERROR(node_->get_logger(),
                    "Timeout waiting for fresh /joint_states (%.1f sec)", timeout_sec);
                return false;
            }
        }
        RCLCPP_INFO(node_->get_logger(), "Fresh /joint_states received");
        return true;
    }

    /**
     * 位控模式初始化：先切位控 → 重新激活控制器 → 获取新 /joint_states → 发当前位置锁定
     */
    void callSetPositionMode()
    {
        // Step 0: 清空 joint_commands 队列（防止急停期间的残留命令）
        if (clear_queue_callback_) {
            clear_queue_callback_();
        }

        // Step 1: 切换到位置模式
        if (!motor_mode_client_ || !motor_mode_client_->service_is_ready()) {
            RCLCPP_ERROR(node_->get_logger(), "MotorMode service not available");
            publishRecoveryDone(false);
            return;
        }
        auto request = std::make_shared<cerebellum_sdk_msg::srv::MotorMode::Request>();
        request->mode = 1;  // 位置模式
        auto mode_future = motor_mode_client_->async_send_request(request);
        if (mode_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "MotorMode service call timeout");
            publishRecoveryDone(false);
            return;
        }
        auto mode_result = mode_future.get();
        if (mode_result->error_code != 0) {
            RCLCPP_ERROR(node_->get_logger(), "MotorMode service failed, error_code=%d",
                         mode_result->error_code);
            publishRecoveryDone(false);
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "Position mode activated (mode=1)");

        // Step 2: 等待最新的 /joint_states（不用缓存旧数据）
        if (!waitForFreshJointState(5.0)) {
            RCLCPP_ERROR(node_->get_logger(),
                "No fresh joint state after mode switch, skip position sync");
            publishRecoveryDone(false);
            return;
        }

        // Step 3: 获取刚收到的关节位置
        std::vector<std::string> joint_names;
        std::vector<double> joint_positions;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            for (const auto& kv : current_joint_positions_) {
                joint_names.push_back(kv.first);
                joint_positions.push_back(kv.second);
            }
        }

        if (joint_names.empty()) {
            RCLCPP_WARN(node_->get_logger(), "No joint positions available after mode switch");
            publishRecoveryDone(false);
            return;
        }

        // Step 4: 分左右臂通过 JointPosition action 发送当前位置（走 MoveIt → 控制器完整链路）
        std::vector<std::string> left_names, right_names;
        std::vector<double> left_positions, right_positions;
        for (size_t i = 0; i < joint_names.size(); ++i) {
            if (joint_names[i].find("left_") != std::string::npos) {
                left_names.push_back(joint_names[i]);
                left_positions.push_back(joint_positions[i]);
            } else if (joint_names[i].find("right_") != std::string::npos) {
                right_names.push_back(joint_names[i]);
                right_positions.push_back(joint_positions[i]);
            }
        }

        // 打印获取到的关节位置
        auto logPositions = [this](const std::string& label,
                                   const std::vector<std::string>& names,
                                   const std::vector<double>& positions) {
            std::string pos_str;
            for (size_t i = 0; i < names.size(); ++i) {
                pos_str += names[i] + "=" + std::to_string(positions[i]);
                if (i + 1 < names.size()) pos_str += ", ";
            }
            RCLCPP_INFO(node_->get_logger(), "%s current positions: [%s]", label.c_str(), pos_str.c_str());
        };
        logPositions("Left arm", left_names, left_positions);
        logPositions("Right arm", right_names, right_positions);

        // Step 4: 直接发轨迹给控制器锁定当前位置（不走 MoveIt，避免 getCurrentState 超时崩溃）
        RCLCPP_INFO(node_->get_logger(), "Locking position via direct trajectory (left=%zu, right=%zu joints)",
            left_names.size(), right_names.size());

        auto build_lock_traj = [this](const std::vector<std::string>& names,
                                       const std::vector<double>& positions)
            -> trajectory_msgs::msg::JointTrajectory
        {
            trajectory_msgs::msg::JointTrajectory msg;
            msg.header.stamp = node_->now();
            msg.joint_names = names;
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions = positions;
            pt.velocities.resize(names.size(), 0.0);
            pt.time_from_start = rclcpp::Duration::from_seconds(0.1);
            msg.points.push_back(pt);
            return msg;
        };

        if (left_arm_traj_pub_ && !left_names.empty()) {
            auto traj_msg = build_lock_traj(left_names, left_positions);
            left_arm_traj_pub_->publish(traj_msg);

            // 打印发布的锁定轨迹详情
            std::string lock_str;
            for (size_t i = 0; i < left_names.size(); ++i) {
                lock_str += left_names[i] + "=" + std::to_string(left_positions[i]);
                if (i + 1 < left_names.size()) lock_str += ", ";
            }
            RCLCPP_INFO(node_->get_logger(), "[锁定轨迹] Left arm: [%s]", lock_str.c_str());
        }
        if (right_arm_traj_pub_ && !right_names.empty()) {
            auto traj_msg = build_lock_traj(right_names, right_positions);
            right_arm_traj_pub_->publish(traj_msg);

            // 打印发布的锁定轨迹详情
            std::string lock_str;
            for (size_t i = 0; i < right_names.size(); ++i) {
                lock_str += right_names[i] + "=" + std::to_string(right_positions[i]);
                if (i + 1 < right_names.size()) lock_str += ", ";
            }
            RCLCPP_INFO(node_->get_logger(), "[锁定轨迹] Right arm: [%s]", lock_str.c_str());
        }

        // 锁定轨迹已发送，现在激活控制器（控制器会立即执行锁定轨迹，不会抖动）
        if (switch_controller_cli_ && switch_controller_cli_->service_is_ready()) {
            auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
            req->activate_controllers = {"left_arm_group_controller", "right_arm_group_controller"};
            req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
            auto future = switch_controller_cli_->async_send_request(req);
            if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
                RCLCPP_INFO(node_->get_logger(), "Controllers reactivated after lock trajectory");
            } else {
                RCLCPP_WARN(node_->get_logger(), "Controller reactivation timeout");
            }
        }

        RCLCPP_INFO(node_->get_logger(), "Position mode setup complete");
        publishRecoveryDone(true);
    }

    /**
     * 发布急停恢复完成通知（无论成功还是失败都发布，避免系统卡住）
     */
    void publishRecoveryDone(bool success)
    {
        if (estop_recovery_done_pub_) {
            std_msgs::msg::Bool done_msg;
            done_msg.data = true;
            estop_recovery_done_pub_->publish(done_msg);
            RCLCPP_INFO(node_->get_logger(), "Estop recovery done notification sent (success=%d)", success);
        }
    }

    /**
     * 发送单臂目标并等待完成
     */
    bool sendArmGoal(const std::vector<std::string>& names,
                     const std::vector<double>& positions,
                     const std::string& arm_label)
    {
        if (!joint_position_client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node_->get_logger(), "%s: action server not available", arm_label.c_str());
            return false;
        }

        auto goal_msg = JointPosition::Goal();
        goal_msg.timestamp = node_->now();
        goal_msg.target_state.header.stamp = node_->now();
        goal_msg.target_state.header.frame_id = "world";
        goal_msg.target_state.name = names;
        goal_msg.target_state.position = positions;

        RCLCPP_INFO(node_->get_logger(), "%s: sending initial pose goal", arm_label.c_str());
        auto send_goal_options = rclcpp_action::Client<JointPosition>::SendGoalOptions();
        auto goal_future = joint_position_client_->async_send_goal(goal_msg, send_goal_options);
        if (goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "%s: goal send timeout", arm_label.c_str());
            return false;
        }
        auto goal_handle = goal_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "%s: goal rejected", arm_label.c_str());
            return false;
        }
        auto result_future = joint_position_client_->async_get_result(goal_handle);
        if (result_future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
            RCLCPP_ERROR(node_->get_logger(), "%s: execution timeout", arm_label.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(), "%s: initial pose reached", arm_label.c_str());
        return true;
    }

    /**
     * 开机后发送双臂初始姿态（仅开机时调用，急停恢复不调用）
     */
    void sendInitialPose()
    {
        RCLCPP_INFO(node_->get_logger(), "Sending initial arm poses...");

        // 左臂初始姿态
        std::vector<std::string> left_names = {
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_joint",
            "left_wrist_roll_joint", "left_wrist_yaw_joint",
            "left_wrist_pitch_joint"
        };
        std::vector<double> left_positions = {
            0.7854, -0.6109, -0.3491, -1.5708, 0.2617, 0.0, -0.8727
        };

        // 右臂初始姿态
        std::vector<std::string> right_names = {
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_joint",
            "right_wrist_roll_joint", "right_wrist_yaw_joint",
            "right_wrist_pitch_joint"
        };
        std::vector<double> right_positions = {
            -0.7854, 0.6109, 0.3491, 1.5708, -0.2617, 0.0, 0.8726
        };

        sendArmGoal(left_names, left_positions, "Left arm");
        std::this_thread::sleep_for(std::chrono::seconds(3));
        sendArmGoal(right_names, right_positions, "Right arm");

        RCLCPP_INFO(node_->get_logger(), "Initial arm poses complete");
    }

private:
    rclcpp::Node::SharedPtr node_;
    bool monitor_only_ = false;

    // 关节限位缓存
    std::unordered_map<std::string, PosLimit> joint_pos_limits_;
    mutable std::mutex limits_mutex_;

    // 关节状态缓存
    std::unordered_map<std::string, double> current_joint_positions_;
    mutable std::mutex joint_state_mutex_;

    // 订阅器
    rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<cerebellum_sdk_msg::msg::MotorState>::SharedPtr motor_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cerebellum_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_recovery_done_sub_;

    // Service client
    rclcpp::Client<cerebellum_sdk_msg::srv::MotorMode>::SharedPtr motor_mode_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_cli_;
    rclcpp_action::Client<JointPosition>::SharedPtr joint_position_client_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_commands_pub_;

    // 急停停止轨迹 publisher
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_traj_pub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_traj_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_recovery_done_pub_;

    // 急停状态
    std::atomic<bool> emergency_stop_active_{false};
    std::atomic<int> current_run_mode_{0};
    std::atomic<bool> first_motor_state_received_{false};
    std::atomic<bool> fresh_joint_state_received_{false};
    std::atomic<bool> need_refresh_start_state_{false};

    // 初始化进行中标志（防止并发执行阻塞操作）
    std::atomic<bool> init_in_progress_{false};

    // 三停相关（与 execute_trajectory_server 相同）
    bool invert_three_stop_ = true;
    std::atomic<bool> cerebellum_allow_execute_{false};
    std::chrono::steady_clock::time_point last_cerebellum_msg_time_{std::chrono::steady_clock::now()};
    static constexpr int64_t CEREBELLUM_TIMEOUT_MS = 2500;

    // 急停回调（move_group.stop() 等）
    std::vector<std::function<void()>> emergency_stop_callbacks_;
    mutable std::mutex estop_cb_mutex_;

    // 队列清空回调（急停恢复时同步调用）
    std::function<void()> clear_queue_callback_;

    void jointLimitsCallback(const planning_sdk_msgs::msg::JointLimitsArray::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(limits_mutex_);
        joint_pos_limits_.clear();
        for (const auto& jl : msg->limits)
        {
            joint_pos_limits_[jl.joint_name] = PosLimit{
                jl.lower_limit,
                jl.upper_limit,
                jl.velocity_limit,
                jl.effort_limit
            };
        }
        RCLCPP_DEBUG(node_->get_logger(), "Updated joint position limits (%zu joints)", msg->limits.size());
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i)
        {
            current_joint_positions_[msg->name[i]] = msg->position[i];
        }
        fresh_joint_state_received_.store(true);
    }

    void motorStatesCallback(const cerebellum_sdk_msg::msg::MotorState::SharedPtr msg)
    {
        // 确认回调被调用（每5秒打印一次）
        static auto last_log_time = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 5) {
            RCLCPP_INFO(node_->get_logger(), "[motor_states回调] 正在接收消息 (mode=%d)",
                msg->run_mode.size() > 0 ? msg->run_mode[0] : 0);
            last_log_time = now;
        }

        // run_mode is an array, check first element as overall state
        uint8_t current_mode = (msg->run_mode.size() > 0) ? msg->run_mode[0] : 0;
        int prev_mode = current_run_mode_.load();
        current_run_mode_.store(current_mode);

        // 调试日志：记录所有模式变化
        if (current_mode != prev_mode) {
            RCLCPP_INFO(node_->get_logger(), "Motor mode changed: %d -> %d", prev_mode, current_mode);
        }

        if (current_mode == 7 && prev_mode != 7) {
            // 触发急停：设置标志 + 调用所有注册的急停回调
            emergency_stop_active_.store(true);
            if (!monitor_only_) {
                sendStopTrajectories();  // 先发停止轨迹覆盖控制器缓冲区
                haltControllers();
            }
            {
                std::lock_guard<std::mutex> lock(estop_cb_mutex_);
                for (auto& cb : emergency_stop_callbacks_) {
                    try { cb(); } catch (...) {}
                }
            }
            RCLCPP_WARN(node_->get_logger(), "EMERGENCY STOP TRIGGERED (run_mode=7)%s",
                monitor_only_ ? "" : ", controllers halted, move_groups stopped");
        }
        else if (current_mode != 7 && prev_mode == 7) {
            if (monitor_only_) {
                // 监控模式：只清标志
                emergency_stop_active_.store(false);
                need_refresh_start_state_.store(true);
                RCLCPP_INFO(node_->get_logger(), "Emergency stop cleared (run_mode=%d)", current_mode);
            } else {
                // 完整模式：在独立线程中切位控 + 锁定当前位置
                RCLCPP_INFO(node_->get_logger(), "Emergency stop recovering (run_mode %d->%d), requesting position mode (async)...",
                    prev_mode, current_mode);
                std::thread([this]() {
                    if (init_in_progress_.exchange(true)) {
                        RCLCPP_WARN(node_->get_logger(), "Init already in progress, skipping estop recovery");
                        return;
                    }
                    callSetPositionMode();
                    emergency_stop_active_.store(false);
                    need_refresh_start_state_.store(true);
                    init_in_progress_.store(false);
                    RCLCPP_INFO(node_->get_logger(), "Emergency stop recovery complete");
                }).detach();
            }
        }
        else if (current_mode == 0 && !first_motor_state_received_.exchange(true)) {
            if (monitor_only_) {
                // 监控模式：跳过初始化动作
                RCLCPP_INFO(node_->get_logger(), "First motor state received (monitor-only, no init action)");
            } else {
                // 完整模式：在独立线程中切位控 + 锁定当前位置 + 发初始姿态
                RCLCPP_INFO(node_->get_logger(),
                    "First motor state received (mode=0), syncing position and entering position mode (async)...");
                std::thread([this]() {
                    if (init_in_progress_.exchange(true)) {
                        RCLCPP_WARN(node_->get_logger(), "Init already in progress, skipping first-boot init");
                        return;
                    }
                    callSetPositionMode();
                    sendInitialPose();
                    init_in_progress_.store(false);
                    RCLCPP_INFO(node_->get_logger(), "First-boot initialization complete");
                }).detach();
            }
        }
    }
};

}  // namespace basic_control_topic