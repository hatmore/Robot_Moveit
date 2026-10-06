#pragma once
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <mutex>
#include <unordered_map>
#include <cmath>
#include <future>

class JointStateMonitor {
public:
  JointStateMonitor(rclcpp::Node* node, const std::string& topic = "/cerebellum_sdk/arm/joint_states")
    : node_(node), monitoring_(false) {
    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          positions_[msg->name[i]] = msg->position[i];
        }
        last_update_time_ = node_->get_clock()->now();
        received_ = true;
      });
  }

  ~JointStateMonitor() {
    stopMonitoring();
  }

  // 启动监控：检测关节是否在动
  void startMonitoring(const std::vector<std::string>& joint_names, double timeout_sec = 3.0) {
    stopMonitoring();
    monitoring_ = true;
    failed_ = false;
    RCLCPP_INFO(node_->get_logger(), "启动关节监控，超时: %.1f 秒", timeout_sec);
    monitor_thread_ = std::thread([this, joint_names, timeout_sec]() {
      std::unordered_map<std::string, double> last_positions;
      auto start_time = node_->get_clock()->now();
      bool initialized = false;

      while (monitoring_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        double total_movement = 0.0;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          for (const auto& name : joint_names) {
            auto it = positions_.find(name);
            if (it != positions_.end()) {
              if (initialized) {
                auto last_it = last_positions.find(name);
                if (last_it != last_positions.end()) {
                  double diff = it->second - last_it->second;
                  total_movement += diff * diff;
                }
              }
              last_positions[name] = it->second;
            }
          }
          if (!initialized && !last_positions.empty()) {
            initialized = true;
            start_time = node_->get_clock()->now();
          }
        }

        if (!initialized) continue;

        total_movement = std::sqrt(total_movement);
        bool any_moving = (total_movement > 0.05);  // 提高阈值到 0.05 rad

        auto elapsed = (node_->get_clock()->now() - start_time).seconds();
        RCLCPP_DEBUG(node_->get_logger(), "监控: 总位移=%.6f, 已过 %.1f 秒", total_movement, elapsed);

        if (!any_moving && elapsed > timeout_sec) {
          RCLCPP_ERROR(node_->get_logger(),
            "关节 %.1f 秒内无运动（总位移=%.6f < 0.05），SDK可能已挂", elapsed, total_movement);
          failed_ = true;
          monitoring_ = false;
          break;
        }
      }
    });
  }

  void stopMonitoring() {
    monitoring_ = false;
    if (monitor_thread_.joinable()) {
      monitor_thread_.join();
    }
  }

  bool hasFailed() const {
    return failed_;
  }

  // 模板方法：带超时的 execute 封装
  template<typename MoveGroupType, typename PlanType>
  bool executeWithTimeout(MoveGroupType& move_group, PlanType& motion_plan, double extra_timeout_sec = 3.0) {
    const auto& points = motion_plan.trajectory_.joint_trajectory.points;
    double trajectory_duration = 0.0;
    if (!points.empty()) {
      trajectory_duration = points.back().time_from_start.sec +
                            points.back().time_from_start.nanosec / 1e9;
    }

    auto execute_future = std::async(std::launch::async, [&]() -> bool {
      auto result = move_group.execute(motion_plan);
      return result == decltype(result)::SUCCESS;
    });

    auto timeout = std::chrono::duration<double>(trajectory_duration + extra_timeout_sec);
    RCLCPP_INFO(node_->get_logger(), "执行超时设置: 轨迹时长=%.1f秒 + 额外=%.1f秒 = %.1f秒",
                trajectory_duration, extra_timeout_sec, trajectory_duration + extra_timeout_sec);

    if (execute_future.wait_for(timeout) == std::future_status::ready) {
      return execute_future.get();
    } else {
      RCLCPP_ERROR(node_->get_logger(), "执行超时（%.1f秒）,sdk/硬件可能出问题，请检查", trajectory_duration + extra_timeout_sec);
      return false;
    }
  }

  bool verifyExecution(
      const std::vector<std::string>& joint_names,
      const std::vector<double>& target_positions,
      double tolerance = 0.05) {

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_) {
      RCLCPP_ERROR(node_->get_logger(), "未收到 joint_states");
      return false;
    }

    for (size_t i = 0; i < joint_names.size(); ++i) {
      auto it = positions_.find(joint_names[i]);
      if (it == positions_.end()) {
        RCLCPP_ERROR(node_->get_logger(), "无法读取关节[%s]状态", joint_names[i].c_str());
        return false;
      }
      double error = std::abs(it->second - target_positions[i]);
      if (error > tolerance) {
        RCLCPP_ERROR(node_->get_logger(),
          "关节[%s]未到达目标：目标=%.3f, 实际=%.3f, 误差=%.3f",
          joint_names[i].c_str(), target_positions[i], it->second, error);
        return false;
      }
    }
    return true;
  }

private:
  rclcpp::Node* node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  std::unordered_map<std::string, double> positions_;
  rclcpp::Time last_update_time_;
  bool received_ = false;
  std::mutex mutex_;
  std::atomic<bool> monitoring_;
  std::atomic<bool> failed_;
  std::thread monitor_thread_;
};
