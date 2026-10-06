#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <planning_sdk_msgs/action/execute_trajectory.hpp>
#include <planning_sdk_msgs/action/joint_position.hpp>
#include <planning_sdk_msgs/msg/joint_limits_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <cerebellum_sdk_msg/msg/motor_state.hpp>
#include <fstream>
#include "basic_control_topic/shared_resources.hpp"
#include <memory>
#include <thread>
#include <vector>
#include <sstream>
#include <atomic>
#include <string>
#include <map>
#include <cmath>
#include <atomic>
#include <chrono>
#include "basic_control_topic/joint_state_monitor.hpp"

using namespace std::chrono_literals;
using ExecuteTrajectory = planning_sdk_msgs::action::ExecuteTrajectory;
using GoalHandleExecuteTrajectory = rclcpp_action::ServerGoalHandle<ExecuteTrajectory>;
using JointPosition = planning_sdk_msgs::action::JointPosition;

// 轨迹点数据结构
struct TrajectoryPoint {
    double time;
    std::vector<double> positions;
};

class ExecuteTrajectoryServer : public rclcpp::Node
{
public:
    ExecuteTrajectoryServer()
        : Node("execute_trajectory_server"), js_monitor_(this)
    {
                // 初始化左右臂关节名称
        left_arm_joint_names_ = {
            "left_shoulder_pitch_joint",
            "left_shoulder_roll_joint", 
            "left_shoulder_yaw_joint",
            "left_elbow_joint",
            "left_wrist_roll_joint",
            "left_wrist_yaw_joint",
            "left_wrist_pitch_joint"
        };
        
        right_arm_joint_names_ = {
            "right_shoulder_pitch_joint",
            "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint",
            "right_elbow_joint",
            "right_wrist_roll_joint", 
            "right_wrist_yaw_joint",
            "right_wrist_pitch_joint"
        };
        
        // 创建JointPosition action client（用于过渡到轨迹起点）
        joint_position_client_ = rclcpp_action::create_client<JointPosition>(
            this, "/algorithm/grasp_planning/move/joint_position");

        // 订阅关节状态（用于判断是否需要过渡）
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/cerebellum_sdk/arm/joint_states", 10,  // 原 "/joint_states"
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
                    current_joint_positions_[msg->name[i]] = msg->position[i];
                }
            });

        // 创建轨迹发布器
        left_arm_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/left_arm_group_controller/joint_trajectory", rclcpp::QoS(10));
        
        right_arm_trajectory_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/right_arm_group_controller/joint_trajectory", rclcpp::QoS(10));
        
        // 创建左右臂执行轨迹动作服务器
        left_arm_action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this,
            "/algorithm/grasp_planning/move/left_arm/execute_trajectory",
            std::bind(&ExecuteTrajectoryServer::handle_left_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ExecuteTrajectoryServer::handle_left_arm_cancel, this, std::placeholders::_1),
            std::bind(&ExecuteTrajectoryServer::handle_left_arm_accepted, this, std::placeholders::_1));
        
        right_arm_action_server_ = rclcpp_action::create_server<ExecuteTrajectory>(
            this,
            "/algorithm/grasp_planning/move/right_arm/execute_trajectory",
            std::bind(&ExecuteTrajectoryServer::handle_right_arm_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ExecuteTrajectoryServer::handle_right_arm_cancel, this, std::placeholders::_1),
            std::bind(&ExecuteTrajectoryServer::handle_right_arm_accepted, this, std::placeholders::_1));
        
                // 订阅外部暂停话题
        trajectory_pause_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/algorithm/grasp_planning/move/trajectory_pause", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                external_pause_active_.store(msg->data);
                RCLCPP_INFO(this->get_logger(), "External pause signal: %s", msg->data ? "PAUSE" : "RESUME");
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

        RCLCPP_INFO(this->get_logger(), "Execute Trajectory Server started");
        RCLCPP_INFO(this->get_logger(), "Left arm action: /algorithm/grasp_planning/move/left_arm/execute_trajectory");
        RCLCPP_INFO(this->get_logger(), "Right arm action: /algorithm/grasp_planning/move/right_arm/execute_trajectory");
        RCLCPP_INFO(this->get_logger(), "Left arm publisher: /left_arm_group_controller/joint_trajectory");
        RCLCPP_INFO(this->get_logger(), "Right arm publisher: /right_arm_group_controller/joint_trajectory");
        RCLCPP_INFO(this->get_logger(), "Trajectory control: /cerebellum_sdk/three_stop/status (Bool, true=run, false=pause)");

        // SharedResources 在 main() 中构造完成后初始化（不能在构造函数内调用 shared_from_this()）
    }

    /**
     * 初始化 SharedResources（必须在 std::make_shared 之后调用）
     */
    void initSharedResources()
    {
        shared_resources_ = std::make_shared<basic_control_topic::SharedResources>();
        shared_resources_->initialize(shared_from_this(), /*monitor_only=*/true);

        // 注册急停回调
        shared_resources_->registerEmergencyStopCallback([this]() {
            RCLCPP_WARN(this->get_logger(), "SharedResources estop callback: stopping execution");
            left_arm_executing_ = false;
            right_arm_executing_ = false;
        });

        // 定时打印急停状态（用于诊断）
        status_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                RCLCPP_INFO(this->get_logger(), "[状态监控] estop=%d, need_refresh=%d, run_mode=%d",
                    shared_resources_->isEmergencyStopActive(),
                    shared_resources_->needRefreshStartState(),
                    shared_resources_->getRunMode());
            });
    }

private:
    // Action服务器
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr left_arm_action_server_;
    rclcpp_action::Server<ExecuteTrajectory>::SharedPtr right_arm_action_server_;

    // SharedResources（用于急停和三停统一管理）
    basic_control_topic::SharedResources::Ptr shared_resources_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    // 轨迹发布器
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr left_arm_trajectory_publisher_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr right_arm_trajectory_publisher_;
    
    // 关节名称
    std::vector<std::string> left_arm_joint_names_;
    std::vector<std::string> right_arm_joint_names_;
    
    // 执行状态
    bool left_arm_executing_ = false;
    bool right_arm_executing_ = false;
    bool left_arm_was_paused_ = false;
    bool right_arm_was_paused_ = false;

    // 逐点发布参数：每次发布的轨迹点数（包含当前点）
    static constexpr size_t POINTS_PER_PUBLISH = 3;

    // JointPosition action client
    rclcpp_action::Client<JointPosition>::SharedPtr joint_position_client_;

    // 关节状态订阅
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    
    // 外部轨迹暂停控制（true=暂停，false=继续）
    std::atomic<bool> external_pause_active_{false};
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trajectory_pause_sub_;
    std::mutex joint_state_mutex_;
    std::map<std::string, double> current_joint_positions_;
    JointStateMonitor js_monitor_;

    struct PosLimit { double min_pos; double max_pos; };
    std::map<std::string, PosLimit> soft_limits_;
    std::mutex soft_limits_mutex_;
    rclcpp::Subscription<planning_sdk_msgs::msg::JointLimitsArray>::SharedPtr joint_limits_sub_;

    // 过渡阈值（rad），超过此值则先移动到起点
    static constexpr double POSITION_THRESHOLD = 0.05;

    // 左臂回调函数
    rclcpp_action::GoalResponse handle_left_arm_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        (void)uuid;
        return handle_goal(goal, true);
    }

    rclcpp_action::CancelResponse handle_left_arm_cancel(
        const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        (void)goal_handle;
        return handle_cancel(true);
    }

    void handle_left_arm_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        std::thread{std::bind(&ExecuteTrajectoryServer::execute_trajectory, this, std::placeholders::_1, true), goal_handle}.detach();
    }

    // 右臂回调函数
    rclcpp_action::GoalResponse handle_right_arm_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const ExecuteTrajectory::Goal> goal)
    {
        (void)uuid;
        return handle_goal(goal, false);
    }

    rclcpp_action::CancelResponse handle_right_arm_cancel(
        const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        (void)goal_handle;
        return handle_cancel(false);
    }

    void handle_right_arm_accepted(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle)
    {
        std::thread{std::bind(&ExecuteTrajectoryServer::execute_trajectory, this, std::placeholders::_1, false), goal_handle}.detach();
    }

    // 通用的目标处理函数
    rclcpp_action::GoalResponse handle_goal(
        std::shared_ptr<const ExecuteTrajectory::Goal> goal, bool is_left_arm)
    {
        std::string arm_name = is_left_arm ? "Left arm" : "Right arm";

        // 检查急停状态
        if (shared_resources_->isEmergencyStopActive()) {
            RCLCPP_WARN(this->get_logger(), "%s rejecting goal: emergency stop active", arm_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        if ((is_left_arm && left_arm_executing_) || (!is_left_arm && right_arm_executing_)) {
            RCLCPP_WARN(this->get_logger(), "%s already executing a trajectory, rejecting new goal", arm_name.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        std::ifstream file(goal->trajectory_file);
        if (!file.good()) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory file not found: %s", goal->trajectory_file.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        
        RCLCPP_INFO(this->get_logger(), "%s trajectory execution goal accepted: %s", arm_name.c_str(), goal->trajectory_file.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // 通用的取消处理函数
    rclcpp_action::CancelResponse handle_cancel(bool is_left_arm)
    {
        std::string arm_name = is_left_arm ? "Left arm" : "Right arm";
        RCLCPP_INFO(this->get_logger(), "%s trajectory execution cancellation requested", arm_name.c_str());
        
        if (is_left_arm) {
            left_arm_executing_ = false;
        } else {
            right_arm_executing_ = false;
        }
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }

        std::vector<TrajectoryPoint> parse_csv_trajectory(const std::string& file_path, const std::vector<std::string>& joint_names, uint16_t velocity) {
        std::vector<TrajectoryPoint> trajectory;
        std::ifstream file(file_path);
        std::string line;
        
        // 读取标题行
        if (!std::getline(file, line)) {
            throw std::runtime_error("Failed to read header from CSV file");
        }
        
        RCLCPP_DEBUG(this->get_logger(), "CSV header: %s", line.c_str());
        
        // 读取数据行
        int line_count = 0;
        while (std::getline(file, line)) {
            line_count++;
            std::stringstream ss(line);
            TrajectoryPoint point;
            std::string value;
            
            // 读取时间
            if (!std::getline(ss, value, ',')) {
                RCLCPP_WARN(this->get_logger(), "Skipping malformed line %d: %s", line_count, line.c_str());
                continue;
            }
            // 防御性检查：velocity 不能为 0 或过小
            if (velocity == 0 || velocity < 1) {
                RCLCPP_ERROR(this->get_logger(),
                             "Invalid velocity %u in ExecuteTrajectory goal, must be >= 1. Using default 5.",
                             velocity);
                velocity = 5;
            }
            static constexpr double MAX_VELOCITY_TIME = 0.08;
            point.time = std::stod(value) * MAX_VELOCITY_TIME * (100.0 / velocity);
            
            // 读取关节位置
            int joint_count = 0;
            while (std::getline(ss, value, ',')) {
                if (!value.empty()) {
                    point.positions.push_back(std::stod(value));
                    joint_count++;
                }
            }
            
            if (point.positions.size() != joint_names.size()) {
                RCLCPP_WARN(this->get_logger(), 
                           "Mismatch in joint count at time %.3f: expected %zu, got %zu", 
                           point.time, joint_names.size(), point.positions.size());
                continue;
            }
            
            trajectory.push_back(point);
        }
        
        RCLCPP_INFO(this->get_logger(), "Successfully parsed %zu trajectory points", trajectory.size());
        
        return trajectory;
    }

    // 判断当前是否允许执行轨迹（三停 + 外部暂停，任一暂停则不允许）
    // 三停：true=停止，false=允许
    // 外部暂停：true=暂停，false=继续
    bool is_execution_allowed() const
    {
        // 使用 SharedResources 的三停检查
        bool three_stop_ok = shared_resources_->isThreeStopAllowExecute();

        // 外部暂停信号（true=暂停，false=继续）
        bool external_paused = external_pause_active_.load();

        // 三停允许 且 外部未暂停 → 允许执行
        return three_stop_ok && !external_paused;
    }

    void execute_trajectory(const std::shared_ptr<GoalHandleExecuteTrajectory> goal_handle, bool is_left_arm)
    {
        std::string arm_name = is_left_arm ? "Left arm" : "Right arm";

        // 急停检查
        if (shared_resources_->isEmergencyStopActive()) {
            RCLCPP_WARN(this->get_logger(), "%s: Emergency stop active, aborting", arm_name.c_str());
            auto result = std::make_shared<ExecuteTrajectory::Result>();
            result->timestamp = this->now();
            result->error_code = 5;
            goal_handle->abort(result);
            return;
        }

        // 急停恢复后刷新位置
        if (shared_resources_->needRefreshStartState()) {
            RCLCPP_WARN(this->get_logger(), "%s: Waiting for emergency stop recovery to complete...", arm_name.c_str());

            // 等待 basic_control_node 完成恢复（最多10秒）
            auto start_time = this->now();
            while (shared_resources_->needRefreshStartState() &&
                   (this->now() - start_time).seconds() < 10.0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }

            if (shared_resources_->needRefreshStartState()) {
                RCLCPP_ERROR(this->get_logger(), "%s: Timeout waiting for recovery, aborting", arm_name.c_str());
                auto result = std::make_shared<ExecuteTrajectory::Result>();
                result->timestamp = this->now();
                result->error_code = 99;
                goal_handle->abort(result);
                return;
            }

            RCLCPP_INFO(this->get_logger(), "%s: Emergency stop recovery complete, proceeding", arm_name.c_str());
        }

        if (is_left_arm) {
            left_arm_executing_ = true;
        } else {
            right_arm_executing_ = true;
        }

        auto result = std::make_shared<ExecuteTrajectory::Result>();
        auto feedback = std::make_shared<ExecuteTrajectory::Feedback>();
        const auto goal = goal_handle->get_goal();

        RCLCPP_INFO(this->get_logger(), "Starting %s trajectory execution: %s", arm_name.c_str(), goal->trajectory_file.c_str());
        
        try {
            auto joint_names = is_left_arm ? left_arm_joint_names_ : right_arm_joint_names_;
            
            // 解析CSV轨迹文件
            auto trajectory_points = parse_csv_trajectory(goal->trajectory_file, joint_names, goal->velocity);
            if (trajectory_points.empty()) {
                throw std::runtime_error("No valid trajectory points found in file");
            }

            // 安全过渡：检查当前位置与轨迹起点的偏差
            if (!move_to_start_if_needed(trajectory_points[0], joint_names, arm_name)) {
                throw std::runtime_error("Failed to move to trajectory start position for " + arm_name);
            }
            
            // Verify all trajectory points are within soft limits before execution
            {
                std::lock_guard<std::mutex> lock(soft_limits_mutex_);
                bool violated = false;
                for (const auto& pt : trajectory_points) {
                    for (size_t i = 0; i < joint_names.size() && i < pt.positions.size(); ++i) {
                        auto lit = soft_limits_.find(joint_names[i]);
                        if (lit != soft_limits_.end()) {
                            if (pt.positions[i] < lit->second.min_pos || pt.positions[i] > lit->second.max_pos) {
                                RCLCPP_ERROR(this->get_logger(),
                                    "%s: Trajectory REJECTED [%s] pos=%.3f not in [%.3f, %.3f] at t=%.3f",
                                    arm_name.c_str(), joint_names[i].c_str(), pt.positions[i],
                                    lit->second.min_pos, lit->second.max_pos, pt.time);
                                violated = true;
                            }
                        }
                    }
                }
                if (violated) {
                    result->timestamp = this->now();
                    result->error_code = 4;
                    goal_handle->abort(result);
                    if (is_left_arm) left_arm_executing_ = false;
                    else right_arm_executing_ = false;
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "%s: All %zu trajectory points passed soft limit check",
                            arm_name.c_str(), trajectory_points.size());
            }

            auto publisher = is_left_arm ? left_arm_trajectory_publisher_ : right_arm_trajectory_publisher_;
            double total_duration = trajectory_points.back().time;

            RCLCPP_INFO(this->get_logger(), "Starting point-by-point trajectory execution for %s with %zu points",
                       arm_name.c_str(), trajectory_points.size());
            
            // 逐点发布轨迹
            for (size_t i = 0; i < trajectory_points.size() && rclcpp::ok(); ++i) {
                if (goal_handle->is_canceling()) {
                    result->timestamp = this->now();
                    result->error_code = 1; // Cancelled
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "%s trajectory execution cancelled by user", arm_name.c_str());
                    if (is_left_arm) {
                        left_arm_executing_ = false;
                    } else {
                        right_arm_executing_ = false;
                    }
                    return;
                }
                
                // 检查执行状态
                if ((is_left_arm && !left_arm_executing_) || (!is_left_arm && !right_arm_executing_)) {
                    RCLCPP_INFO(this->get_logger(), "%s trajectory execution stopped", arm_name.c_str());
                    return;
                }
                
                // 检查暂停状态（由 /cerebellum_sdk/three_stop/status 控制）
                bool is_paused = !is_execution_allowed();
                bool& was_paused = is_left_arm ? left_arm_was_paused_ : right_arm_was_paused_;

                if (is_paused && !was_paused) {
                    was_paused = true;
                    // 发送停止轨迹（只有当前位置，速度为0）
                    auto stop_trajectory = build_stop_trajectory(trajectory_points[i], joint_names);
                    publisher->publish(stop_trajectory);
                    RCLCPP_INFO(this->get_logger(), "%s trajectory paused at point %zu (cerebellum stop signal)",
                                arm_name.c_str(), i);
                }

                // 等待 resume
                while (is_paused && rclcpp::ok()) {
                    // 在暂停等待期间也要检查急停，确保急停能立即生效
                    if (shared_resources_->isEmergencyStopActive()) {
                        RCLCPP_WARN(this->get_logger(), "%s trajectory aborted due to emergency stop during pause at point %zu",
                                    arm_name.c_str(), i);
                        result->timestamp = this->now();
                        result->error_code = 5;
                        goal_handle->abort(result);
                        if (is_left_arm) {
                            left_arm_executing_ = false;
                        } else {
                            right_arm_executing_ = false;
                        }
                        return;
                    }
                    std::this_thread::sleep_for(100ms);
                    is_paused = !is_execution_allowed();
                }

                if (was_paused && !is_paused) {
                    was_paused = false;
                    RCLCPP_INFO(this->get_logger(), "%s trajectory resumed from point %zu", arm_name.c_str(), i);
                }

                // 急停检查（使用 SharedResources）
                if (shared_resources_->isEmergencyStopActive()) {
                    RCLCPP_WARN(this->get_logger(), "%s trajectory aborted due to emergency stop at point %zu", arm_name.c_str(), i);
                    result->timestamp = this->now();
                    result->error_code = 5;  // 急停错误码
                    goal_handle->abort(result);
                    if (is_left_arm) {
                        left_arm_executing_ = false;
                    } else {
                        right_arm_executing_ = false;
                    }
                    return;
                }

                // 发布当前点及后续几个点
                size_t end_idx = std::min(i + POINTS_PER_PUBLISH, trajectory_points.size());
                std::vector<TrajectoryPoint> points_to_publish(
                    trajectory_points.begin() + i,
                    trajectory_points.begin() + end_idx);

                // 调整时间从0开始
                double time_offset = trajectory_points[i].time;
                for (auto& pt : points_to_publish) {
                    pt.time -= time_offset;
                }

                auto segment_trajectory = build_trajectory_message(points_to_publish, joint_names);
                publisher->publish(segment_trajectory);

                RCLCPP_DEBUG(this->get_logger(), "%s: published points %zu-%zu",
                           arm_name.c_str(), i, end_idx - 1);
                
                // 计算并发布进度
                const auto& point = trajectory_points[i];
                int progress = static_cast<int>((point.time / total_duration) * 100);
                progress = std::min(progress, 100);
                
                feedback->timestamp = this->now();
                feedback->process_status = progress;
                goal_handle->publish_feedback(feedback);
                
                // 等待到下一个点的时间
                if (i < trajectory_points.size() - 1) {
                    double time_to_next = trajectory_points[i + 1].time - point.time;
                    std::this_thread::sleep_for(std::chrono::duration<double>(time_to_next));
                }
            }
            
            // 检查执行状态
            if ((is_left_arm && !left_arm_executing_) || (!is_left_arm && !right_arm_executing_)) {
                RCLCPP_INFO(this->get_logger(), "%s trajectory execution stopped", arm_name.c_str());
                return;
            }

            // 验证是否真的到达目标
            auto& last_point = trajectory_points.back();
            if (!js_monitor_.verifyExecution(joint_names, last_point.positions)) {
                RCLCPP_ERROR(this->get_logger(), "%s trajectory execution verification failed, SDK may be down", arm_name.c_str());
                result->timestamp = this->now();
                result->error_code = 3;
                goal_handle->abort(result);
                if (is_left_arm) {
                    left_arm_executing_ = false;
                } else {
                    right_arm_executing_ = false;
                }
                return;
            }
            
            // 执行成功
            result->timestamp = this->now();
            result->error_code = 0;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "%s trajectory execution completed successfully", arm_name.c_str());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "%s trajectory execution failed: %s", arm_name.c_str(), e.what());
            
            result->timestamp = this->now();
            result->error_code = 2; // Execution failed
            goal_handle->abort(result);
        }
        
        if (is_left_arm) {
            left_arm_executing_ = false;
        } else {
            right_arm_executing_ = false;
        }
    }

    // 检查当前位置与轨迹起点偏差，超过阈值则调用JointPosition action过渡
    bool move_to_start_if_needed(
        const TrajectoryPoint& start_point,
        const std::vector<std::string>& joint_names,
        const std::string& arm_name)
    {
        // 急停检查（使用 SharedResources）
        if (shared_resources_->isEmergencyStopActive()) {
            RCLCPP_WARN(this->get_logger(), "Emergency stop active, skipping transition for %s", arm_name.c_str());
            return false;
        }

        // 获取当前关节位置
        std::vector<double> current_pos;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            for (const auto& name : joint_names) {
                auto it = current_joint_positions_.find(name);
                if (it == current_joint_positions_.end()) {
                    RCLCPP_WARN(this->get_logger(),
                        "%s: joint_states not available yet, skipping transition check", arm_name.c_str());
                    return true;  // 没有关节状态数据，跳过检查
                }
                current_pos.push_back(it->second);
            }
        }

        // 计算最大偏差
        double max_dev = 0.0;
        for (size_t i = 0; i < current_pos.size(); ++i) {
            max_dev = std::max(max_dev, std::fabs(current_pos[i] - start_point.positions[i]));
        }

        RCLCPP_INFO(this->get_logger(),
            "%s: max deviation from trajectory start: %.4f rad (threshold: %.4f)",
            arm_name.c_str(), max_dev, POSITION_THRESHOLD);

        if (max_dev <= POSITION_THRESHOLD) {
            RCLCPP_INFO(this->get_logger(),
                "%s: already near trajectory start, no transition needed", arm_name.c_str());
            return true;
        }

        // 偏差超过阈值，调用JointPosition action移动到起点
        RCLCPP_INFO(this->get_logger(),
            "%s: deviation %.4f rad > threshold, moving to trajectory start via JointPosition action",
            arm_name.c_str(), max_dev);

        return call_joint_position_action(start_point.positions, joint_names, arm_name);
    }

    bool call_joint_position_action(
        const std::vector<double>& target_positions,
        const std::vector<std::string>& joint_names,
        const std::string& arm_name)
    {
        RCLCPP_INFO(this->get_logger(), "%s: [moveToStart] Entering call_joint_position_action", arm_name.c_str());

        if (!joint_position_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(),
                "%s: JointPosition action server not available", arm_name.c_str());
            return false;
        }

        // 构建goal
        auto goal_msg = JointPosition::Goal();
        goal_msg.timestamp = this->now();
        goal_msg.target_state.header.stamp = this->now();
        goal_msg.target_state.header.frame_id = "world";
        goal_msg.target_state.name = joint_names;
        goal_msg.target_state.position = target_positions;

        RCLCPP_INFO(this->get_logger(),
            "%s: [moveToStart] Sending JointPosition goal", arm_name.c_str());

        auto send_goal_options = rclcpp_action::Client<JointPosition>::SendGoalOptions();
        auto goal_future = joint_position_client_->async_send_goal(goal_msg, send_goal_options);

        // 等待goal被接受
        RCLCPP_INFO(this->get_logger(), "%s: [moveToStart] Waiting for goal acceptance (10s timeout)...", arm_name.c_str());
        auto goal_status = goal_future.wait_for(std::chrono::seconds(10));
        RCLCPP_INFO(this->get_logger(), "%s: [moveToStart] Goal wait completed, estop=%d",
            arm_name.c_str(), shared_resources_->isEmergencyStopActive());

        if (goal_status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(),
                "%s: JointPosition goal send timeout", arm_name.c_str());
            return false;
        }

        auto goal_handle = goal_future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(),
                "%s: JointPosition goal was rejected", arm_name.c_str());
            return false;
        }

        // 等待执行结果
        RCLCPP_INFO(this->get_logger(), "%s: [moveToStart] Waiting for execution result (30s timeout)...", arm_name.c_str());
        auto result_future = joint_position_client_->async_get_result(goal_handle);
        auto result_status = result_future.wait_for(std::chrono::seconds(30));
        RCLCPP_INFO(this->get_logger(), "%s: [moveToStart] Execution wait completed, estop=%d",
            arm_name.c_str(), shared_resources_->isEmergencyStopActive());

        if (result_status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(),
                "%s: JointPosition execution timeout", arm_name.c_str());
            return false;
        }

        auto result = result_future.get();
        if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_ERROR(this->get_logger(),
                "%s: JointPosition execution failed with code %d",
                arm_name.c_str(), static_cast<int>(result.code));
            return false;
        }

        RCLCPP_INFO(this->get_logger(),
            "%s: successfully moved to trajectory start position", arm_name.c_str());
        return true;
    }

    trajectory_msgs::msg::JointTrajectory build_stop_trajectory(
        const TrajectoryPoint& current_point, const std::vector<std::string>& joint_names)
    {
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msg.header.stamp = this->now();
        trajectory_msg.joint_names = joint_names;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = current_point.positions;
        point.velocities.resize(joint_names.size(), 0.0);
        point.time_from_start = rclcpp::Duration::from_seconds(0.1);
        trajectory_msg.points.push_back(point);

        return trajectory_msg;
    }

    trajectory_msgs::msg::JointTrajectory build_trajectory_message(const std::vector<TrajectoryPoint>& trajectory_points, 
                                                                  const std::vector<std::string>& joint_names) {
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        
        // 设置关节名称
        trajectory_msg.joint_names = joint_names;
        
        // 设置头信息
        trajectory_msg.header.stamp = this->now();
        trajectory_msg.header.frame_id = "base_link";
        
        // 构建轨迹点，确保时间严格递增
        static constexpr double MIN_TIME_STEP = 0.001;  // 最小时间步长 1ms
        double last_time = -MIN_TIME_STEP;

        for (const auto& point : trajectory_points) {
            trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
            
            // 设置位置
            trajectory_point.positions = point.positions;

            // 确保时间严格递增
            double t = point.time;
            if (t <= last_time) {
                t = last_time + MIN_TIME_STEP;
                RCLCPP_WARN(this->get_logger(),
                    "Adjusted non-increasing time: %.6f -> %.6f", point.time, t);
            }
            last_time = t;
            
            // 设置时间从开始
            trajectory_point.time_from_start = rclcpp::Duration::from_seconds(t);
            
            trajectory_msg.points.push_back(trajectory_point);
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Built trajectory with %zu points, total duration: %.3f seconds",
                    trajectory_msg.points.size(),
                    static_cast<double>(trajectory_msg.points.back().time_from_start.sec) +
                    trajectory_msg.points.back().time_from_start.nanosec * 1e-9);
        
        return trajectory_msg;
    }

    // 急停时立即向两个控制器发送停止指令（当前位置 + 零速度）
    void sendStopToControllers()
    {
        // 左臂停止
        {
            trajectory_msgs::msg::JointTrajectory stop_msg;
            stop_msg.header.stamp = this->now();
            stop_msg.joint_names = left_arm_joint_names_;
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                for (const auto& name : left_arm_joint_names_) {
                    auto it = current_joint_positions_.find(name);
                    pt.positions.push_back(it != current_joint_positions_.end() ? it->second : 0.0);
                }
            }
            pt.velocities.resize(left_arm_joint_names_.size(), 0.0);
            pt.time_from_start = rclcpp::Duration::from_seconds(0.01);
            stop_msg.points.push_back(pt);
            left_arm_trajectory_publisher_->publish(stop_msg);
        }

        // 右臂停止
        {
            trajectory_msgs::msg::JointTrajectory stop_msg;
            stop_msg.header.stamp = this->now();
            stop_msg.joint_names = right_arm_joint_names_;
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            {
                std::lock_guard<std::mutex> lock(joint_state_mutex_);
                for (const auto& name : right_arm_joint_names_) {
                    auto it = current_joint_positions_.find(name);
                    pt.positions.push_back(it != current_joint_positions_.end() ? it->second : 0.0);
                }
            }
            pt.velocities.resize(right_arm_joint_names_.size(), 0.0);
            pt.time_from_start = rclcpp::Duration::from_seconds(0.01);
            stop_msg.points.push_back(pt);
            right_arm_trajectory_publisher_->publish(stop_msg);
        }

        RCLCPP_WARN(this->get_logger(), "Emergency stop: sent stop trajectories to both arm controllers");
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // 创建单个执行轨迹服务器节点
    auto trajectory_server = std::make_shared<ExecuteTrajectoryServer>();
    trajectory_server->initSharedResources();  // 构造完成后才能调用 shared_from_this()
    
    // 使用多线程执行器（子线程中调用action client需要executor并发处理回调）
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(trajectory_server);
    
    RCLCPP_INFO(trajectory_server->get_logger(), "Execute Trajectory Server started successfully");
    
    executor.spin();
    rclcpp::shutdown();
    
    return 0;
}
