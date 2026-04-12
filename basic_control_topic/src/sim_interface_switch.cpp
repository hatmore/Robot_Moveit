#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <std_msgs/msg/bool.hpp>
#include <planning_sdk_msgs/msg/tcp_pose.hpp>
#include <cerebellum_sdk_msg/msg/motor_state.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <cmath>
#include <unordered_map>
#include <string>
#include <thread>
#include <atomic>

// URDF 硬限位，用于归一化实机多圈旋转导致的超限关节值
static const std::unordered_map<std::string, std::pair<double, double>> URDF_LIMITS = {
    {"left_shoulder_pitch_joint",  {-4.36332,   1.22173}},
    {"left_shoulder_roll_joint",   {-0.733038,  2.879793}},
    {"left_shoulder_yaw_joint",    {-2.7925268, 2.7925268}},
    {"left_elbow_joint",           {-2.338741,  2.338741}},
    {"left_wrist_roll_joint",      {-2.8448867, 2.8448867}},
    {"left_wrist_yaw_joint",       {-0.959931,  0.959931}},
    {"left_wrist_pitch_joint",     {-1.570796,  1.570796}},
    {"right_shoulder_pitch_joint", {-1.22173,   4.36332}},
    {"right_shoulder_roll_joint",  {-2.879793,  0.733038}},
    {"right_shoulder_yaw_joint",   {-2.7925268, 2.7925268}},
    {"right_elbow_joint",          {-2.338741,  2.338741}},
    {"right_wrist_roll_joint",     {-2.8448867, 2.8448867}},
    {"right_wrist_yaw_joint",      {-0.959931,  0.959931}},
    {"right_wrist_pitch_joint",    {-1.570796,  1.570796}},
};

static double normalizeJointPosition(const std::string & name, double pos)
{
  auto it = URDF_LIMITS.find(name);
  if (it == URDF_LIMITS.end()) return pos;
  double lo = it->second.first;
  double hi = it->second.second;
  while (pos < lo) pos += 2.0 * M_PI;
  while (pos > hi) pos -= 2.0 * M_PI;
  if (pos < lo || pos > hi) {
    pos = std::max(lo, std::min(hi, pos));
  }
  return pos;
}

class InterfaceSwitchNode : public rclcpp::Node
{
public:
  InterfaceSwitchNode() : Node("interface_switch_node"), use_simulator_(false)
  {
    // 创建服务
    switch_service_ = this->create_service<std_srvs::srv::SetBool>(
      "/algorithm/grasp_planning/simulator/mode",
      std::bind(&InterfaceSwitchNode::handleSwitchRequest, this, std::placeholders::_1, std::placeholders::_2));

    // 控制器切换客户端（用于模式切换时安全重启控制器，防止飞车）
    switch_controller_cli_ = this->create_client<controller_manager_msgs::srv::SwitchController>(
      "/controller_manager/switch_controller");

    // 统一输出话题（供 MoveIt 等上层使用）
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SensorDataQoS());
    real_joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/cerebellum_sdk/arm/joint_commands", rclcpp::SensorDataQoS());
    sim_joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/algorithm/grasp_planning/simulator/joint_state", rclcpp::SensorDataQoS());

    // 订阅仿真器的关节状态（需要 joint_state_broadcaster 重映射到 /sim_joint_states）
    sim_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/sim_joint_states", rclcpp::SensorDataQoS(),
      std::bind(&InterfaceSwitchNode::simJointStateCallback, this, std::placeholders::_1));

    // 订阅关节指令（仿真模式回显为状态，真机模式转发到真机）
    joint_command_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_commands", rclcpp::SensorDataQoS(),
      std::bind(&InterfaceSwitchNode::jointCommandCallback, this, std::placeholders::_1));

    // 订阅真机的关节状态
    real_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/cerebellum_sdk/arm/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&InterfaceSwitchNode::realJointStateCallback, this, std::placeholders::_1));

    // 订阅电机状态（急停检测）
    motor_states_sub_ = this->create_subscription<cerebellum_sdk_msg::msg::MotorState>(
      "/cerebellum_sdk/arm/motor_states", 10,
      [this](const cerebellum_sdk_msg::msg::MotorState::SharedPtr msg) {
        uint8_t mode = (msg->run_mode.size() > 0) ? msg->run_mode[0] : 0;
        int prev = current_run_mode_.load();
        current_run_mode_.store(mode);

        if (mode == 7 && prev != 7) {
          // 急停触发：屏蔽 joint_commands 转发
          command_gate_open_.store(false);
          joints_stable_.store(false);
          recovery_done_received_.store(false);
          stable_count_ = 0;
          RCLCPP_WARN(this->get_logger(), "Emergency stop detected, blocking joint_commands forwarding");
        } else if (mode == 0 && prev == 7) {
          // 急停恢复：等 joint_states 稳定 + 收到恢复通知后再放行
          stable_count_ = 0;
          RCLCPP_INFO(this->get_logger(), "Emergency stop cleared, waiting for stable joint_states + recovery notification...");
        }
      });

    // 订阅急停恢复完成通知（来自 basic_control_node 的 callSetPositionMode 完成）
    auto recovery_qos = rclcpp::QoS(1).transient_local();
    estop_recovery_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/algorithm/internal/estop_recovery_done", recovery_qos,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
          recovery_done_received_.store(true);
          RCLCPP_INFO(this->get_logger(), "Received estop recovery done notification");
          tryUnblockCommands();
        }
      });

    // 仿真模式末端位姿发布（仅 use_simulator_=true 时发布，只读 TF，不碰 joint_states/joint_commands）
    left_sim_tcp_pub_ = this->create_publisher<planning_sdk_msgs::msg::TcpPose>(
      "/algorithm/grasp_planning/left_arm/simulator/tcp_pose", rclcpp::SensorDataQoS());
    right_sim_tcp_pub_ = this->create_publisher<planning_sdk_msgs::msg::TcpPose>(
      "/algorithm/grasp_planning/right_arm/simulator/tcp_pose", rclcpp::SensorDataQoS());

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    sim_tcp_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10Hz
      std::bind(&InterfaceSwitchNode::publishSimTcpPose, this));

    RCLCPP_INFO(this->get_logger(), "Interface switch node initialized");
  }

private:
  void handleSwitchRequest(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    bool new_sim = request->data;
    if (new_sim == use_simulator_) {
      response->success = true;
      response->message = new_sim ? "Already in SIMULATOR mode" : "Already in REAL ROBOT mode";
      return;
    }

    if (new_sim) {
      // 实机 → 仿真：先停控制器，再切模式，防止控制器残余指令被 echo 到 /joint_states 导致飞车
      RCLCPP_INFO(this->get_logger(), "Switching to SIMULATOR mode, safe handover in progress...");
      std::thread([this]() {
        // Step1: 停控制器，防止残余指令污染 /joint_states
        deactivateControllers();
        // Step2: 切换模式，simJointStateCallback 开始向 /joint_states 注入仿真状态
        use_simulator_ = true;
        // Step3: 等待 TopicBasedSystem 从 /joint_states 刷新硬件接口（约100ms）
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // Step4: 重启控制器，从当前位置初始化，误差=0
        activateControllers();
        RCLCPP_INFO(this->get_logger(), "Safe handover to SIMULATOR complete");
      }).detach();
      response->message = "Switching to simulator interface (safe handover in progress)";
    } else {
      // 仿真 → 真机：先停控制器，切模式等真机状态稳定，再重启控制器，防止飞车
      RCLCPP_INFO(this->get_logger(), "Switching to REAL ROBOT mode, restarting controllers for safe handover...");
      std::thread([this]() {
        // Step1: 停控制器，清空内部积分状态
        deactivateControllers();
        // Step2: 切换模式，realJointStateCallback 开始向 /joint_states 注入真机状态
        use_simulator_ = false;
        // Step3: 等待真机状态通过 TopicBasedSystem 刷新到硬件接口（约200ms）
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // Step4: 重启控制器，从真机当前位置初始化，误差=0，不产生突发指令
        activateControllers();
        RCLCPP_INFO(this->get_logger(), "Safe handover to REAL ROBOT complete");
      }).detach();
      response->message = "Switching to real robot interface (safe handover in progress)";
    }

    response->success = true;
  }

  // 仿真器关节状态回调：转发到 /joint_states（仿真模式）和 /algorithm/.../joint_state（始终）
  void simJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto msg_cur = *msg;
    msg_cur.header.stamp = this->now();
    sim_joint_state_pub_->publish(msg_cur);
    if (use_simulator_) {
      joint_state_pub_->publish(msg_cur);
    }
  }

  // 关节指令回调：仿真模式回显为关节状态，真机模式转发到 /cerebellum_sdk/arm/joint_commands
  void jointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    auto msg_cur = *msg;
    msg_cur.header.stamp = this->now();
    if (use_simulator_) {
      joint_state_pub_->publish(msg_cur);
    } else {
      if (!command_gate_open_.load()) {
        RCLCPP_DEBUG(this->get_logger(), "joint_commands blocked during emergency stop");
        return;
      }
      real_joint_command_pub_->publish(msg_cur);
    }
  }

  // 真机关节状态回调：真机模式下归一化后转发到 /joint_states，并处理急停恢复防抖
  void realJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!use_simulator_) {
      auto msg_cur = *msg;
      msg_cur.header.stamp = this->now();
      for (size_t i = 0; i < msg_cur.name.size() && i < msg_cur.position.size(); ++i) {
        double raw = msg_cur.position[i];
        double normalized = normalizeJointPosition(msg_cur.name[i], raw);
        if (std::fabs(normalized - raw) > 1e-6) {
          RCLCPP_DEBUG(this->get_logger(), "Normalize %s: %.4f -> %.4f",
                       msg_cur.name[i].c_str(), raw, normalized);
        }
        msg_cur.position[i] = normalized;
      }
      joint_state_pub_->publish(msg_cur);

      // 急停恢复防抖：等 joint_states 连续稳定后标记
      if (!command_gate_open_.load() && current_run_mode_.load() == 0) {
        if (!last_joint_state_positions_.empty() &&
            last_joint_state_positions_.size() == msg_cur.position.size()) {
          double max_diff = 0.0;
          for (size_t i = 0; i < msg_cur.position.size(); ++i) {
            max_diff = std::max(max_diff, std::fabs(msg_cur.position[i] - last_joint_state_positions_[i]));
          }
          if (max_diff < STABLE_THRESHOLD) {
            ++stable_count_;
          } else {
            stable_count_ = 0;
          }
          if (stable_count_ >= STABLE_FRAMES_REQUIRED && !joints_stable_.load()) {
            joints_stable_.store(true);
            RCLCPP_INFO(this->get_logger(),
              "joint_states stable (%d frames, max_diff < %.4f)",
              STABLE_FRAMES_REQUIRED, STABLE_THRESHOLD);
            tryUnblockCommands();
          }
        }
        last_joint_state_positions_ = msg_cur.position;
      }
    }
  }

  // 两个条件都满足时才放行
  void tryUnblockCommands()
  {
    if (joints_stable_.load() && recovery_done_received_.load() && !command_gate_open_.load()) {
      command_gate_open_.store(true);
      RCLCPP_INFO(this->get_logger(),
        "Both conditions met (joints stable + recovery done), unblocking joint_commands");
    }
  }

  // 停止轨迹控制器（清空内部积分状态）
  void deactivateControllers()
  {
    if (!switch_controller_cli_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_WARN(this->get_logger(), "controller_manager/switch_controller not available, skip deactivate");
      return;
    }
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->deactivate_controllers = TRAJECTORY_CONTROLLERS;
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    auto future = switch_controller_cli_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Controllers deactivated");
    } else {
      RCLCPP_WARN(this->get_logger(), "Deactivate controllers timeout");
    }
  }

  // 重新激活轨迹控制器（从硬件接口当前位置初始化，误差=0）
  void activateControllers()
  {
    if (!switch_controller_cli_->wait_for_service(std::chrono::seconds(3))) {
      RCLCPP_WARN(this->get_logger(), "controller_manager/switch_controller not available, skip activate");
      return;
    }
    auto req = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = TRAJECTORY_CONTROLLERS;
    req->strictness = controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    auto future = switch_controller_cli_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Controllers activated");
    } else {
      RCLCPP_WARN(this->get_logger(), "Activate controllers timeout");
    }
  }

  // 仅重启（用于仿真→仿真方向，保持控制器状态一致）
  void restartControllers()
  {
    deactivateControllers();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    activateControllers();
  }

  // 仿真模式末端位姿：只读 TF，use_simulator_=false 时直接返回，不产生任何输出
  void publishSimTcpPose()
  {
    if (!use_simulator_) return;

    auto lookup_and_publish = [this](
      const std::string & tip_frame,
      rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr & pub)
    {
      planning_sdk_msgs::msg::TcpPose msg;
      msg.timestamp = this->now();
      msg.error_code = 0;
      try {
        auto tf = tf_buffer_->lookupTransform(
          "base_link", tip_frame, tf2::TimePointZero,
          std::chrono::milliseconds(50));
        msg.current_pose.position.x = tf.transform.translation.x;
        msg.current_pose.position.y = tf.transform.translation.y;
        msg.current_pose.position.z = tf.transform.translation.z;
        msg.current_pose.orientation = tf.transform.rotation;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_DEBUG(this->get_logger(), "TF lookup failed for %s: %s",
                     tip_frame.c_str(), ex.what());
        msg.error_code = 1;
      }
      pub->publish(msg);
    };

    lookup_and_publish("left_gripper_base_link",  left_sim_tcp_pub_);
    lookup_and_publish("right_gripper_base_link", right_sim_tcp_pub_);
  }

  static const inline std::vector<std::string> TRAJECTORY_CONTROLLERS = {
    "left_arm_group_controller",
    "right_arm_group_controller",
  };

  // 急停防抖参数
  static constexpr double STABLE_THRESHOLD = 0.005;  // rad，帧间最大偏差阈值
  static constexpr int STABLE_FRAMES_REQUIRED = 5;   // 连续稳定帧数

  bool use_simulator_;
  std::atomic<bool> command_gate_open_{true};          // 急停时关闭，两条件满足后打开
  std::atomic<bool> joints_stable_{false};             // joint_states 连续稳定
  std::atomic<bool> recovery_done_received_{false};    // 收到 callSetPositionMode 完成通知
  std::atomic<int> current_run_mode_{0};
  int stable_count_ = 0;
  std::vector<double> last_joint_state_positions_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr switch_service_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_cli_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr real_joint_command_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr sim_joint_state_pub_;
  rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr left_sim_tcp_pub_;
  rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr right_sim_tcp_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_joint_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr real_joint_state_sub_;
  rclcpp::Subscription<cerebellum_sdk_msg::msg::MotorState>::SharedPtr motor_states_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_recovery_done_sub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr sim_tcp_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterfaceSwitchNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}