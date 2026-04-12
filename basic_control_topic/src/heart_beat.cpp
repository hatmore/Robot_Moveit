#include "rclcpp/rclcpp.hpp"
#include <planning_sdk_msgs/msg/heart_beat.hpp>
#include <std_msgs/msg/int8.hpp>
#include <mutex>

/**
 * ================================================================================
 * 规控心跳节点 (HeartBeat)
 * ================================================================================
 *
 * 发布话题: /algorithm/grasp_planning/heart_beat  (1Hz)
 * 消息类型: planning_sdk_msgs::msg::HeartBeat
 *   - error_code[]: 当前所有错误的错误码（空 = 系统正常）
 *   - error_msg[] : 对应的描述信息
 *
 * ── 监控项 ──────────────────────────────────────────────────────────────────────
 *
 * 【A】Servo 运行状态（订阅话题，实时感知）
 *   源话题: /left_arm_servo_node/status  (std_msgs/Int8)
 *           /right_arm_servo_node/status (std_msgs/Int8)
 *   状态码含义（moveit_servo 定义）:
 *     0 = 正常
 *     1 = 接近奇异点，正在减速
 *     2 = 非常接近奇异点，硬停止
 *     3 = 接近碰撞，正在减速
 *     4 = 检测到碰撞，硬停止
 *     5 = 接近关节限位，停止
 *     6 = 正在离开奇异点，减速中
 *
 * 【B】节点存活检测（每秒通过 count_publishers 检查 Action /_action/status 话题）
 *   注意：这些 Action Server 节点是"常驻 + 按请求激活"型，不会主动广播自身状态，
 *         因此不订阅其话题，而是检查其 Action 内部 status 话题是否有发布者。
 *   被检测节点:
 *     joint_position_server         → /algorithm/grasp_planning/move/joint_position
 *     joint_position_delta_server   → /algorithm/grasp_planning/move/joint_position_delta
 *     execute_trajectory_server     → /algorithm/grasp_planning/move/left_arm/execute_trajectory
 *     tcp_position_action_server    → /algorithm/grasp_planning/move/left_arm/tcp_position
 *     tcp_position_delta_server     → /algorithm/grasp_planning/move/left_arm/tcp_position_delta
 *     tcp_trajectory_action_server  → /algorithm/grasp_planning/move/left_arm/tcp_trajectory
 *     joint_limits_server           → /algorithm/joint_limits/current
 *
 * ── 不监控的节点 ──────────────────────────────────────────────────────────────
 *   joint_velocity_to_servo / tcp_velocity_to_servo / servo_manager
 *   这些是 Servo 链路中间转发节点，其健康状况已通过 Servo status 间接反映。
 *
 * ── 错误码规范 ────────────────────────────────────────────────────────────────
 *   0x0001  左臂Servo节点无响应（5s超时）
 *   0x0002  右臂Servo节点无响应（5s超时）
 *   0x0011  左臂Servo: 接近奇异点，减速
 *   0x0012  左臂Servo: 非常接近奇异点，硬停止
 *   0x0013  左臂Servo: 接近碰撞，减速
 *   0x0014  左臂Servo: 检测到碰撞，硬停止
 *   0x0015  左臂Servo: 接近关节限位
 *   0x0016  左臂Servo: 离开奇异点，减速中
 *   0x0021  右臂Servo: 接近奇异点，减速
 *   0x0022  右臂Servo: 非常接近奇异点，硬停止
 *   0x0023  右臂Servo: 接近碰撞，减速
 *   0x0024  右臂Servo: 检测到碰撞，硬停止
 *   0x0025  右臂Servo: 接近关节限位
 *   0x0026  右臂Servo: 离开奇异点，减速中
 *   0x0101  joint_position_server 不可用
 *   0x0102  joint_position_delta_server 不可用
 *   0x0103  execute_trajectory_server 不可用
 *   0x0104  tcp_position_action_server 不可用
 *   0x0105  tcp_position_delta_action_server 不可用
 *   0x0106  tcp_trajectory_action_server 不可用
 *   0x0107  joint_limits_server 不可用
 * ================================================================================
 */

// 左臂 Servo 错误码基址
static constexpr uint16_t LEFT_BASE  = 0x0010;
// 右臂 Servo 错误码基址
static constexpr uint16_t RIGHT_BASE = 0x0020;
// Servo 连接超时（秒）
static constexpr double SERVO_DISCONNECT_TIMEOUT_SEC = 5.0;

// Action Server 存活检测：通过 _action/status 话题的发布者数量判断
// ROS2 Action Server 在 spin 期间会持续发布该话题（即使没有 goal）
struct NodeCheck {
    const char* action_status_topic;  // <action_name>/_action/status 或 service 话题
    uint16_t    error_code;
    const char* error_msg;
};

static const NodeCheck NODE_CHECKS[] = {
    {
        "/algorithm/grasp_planning/move/joint_position/_action/status",
        0x0101,
        "joint_position_server 不可用"
    },
    {
        "/algorithm/grasp_planning/move/joint_position_delta/_action/status",
        0x0102,
        "joint_position_delta_server 不可用"
    },
    {
        "/algorithm/grasp_planning/move/left_arm/execute_trajectory/_action/status",
        0x0103,
        "execute_trajectory_server 不可用"
    },
    {
        "/algorithm/grasp_planning/move/left_arm/tcp_position/_action/status",
        0x0104,
        "tcp_position_action_server 不可用"
    },
    {
        "/algorithm/grasp_planning/move/left_arm/tcp_position_delta/_action/status",
        0x0105,
        "tcp_position_delta_action_server 不可用"
    },
    {
        "/algorithm/grasp_planning/move/left_arm/tcp_trajectory/_action/status",
        0x0106,
        "tcp_trajectory_action_server 不可用"
    },
    {
        // joint_limits_server 通过其 transient_local 发布话题存活检测
        "/algorithm/joint_limits/current",
        0x0107,
        "joint_limits_server 不可用"
    },
};

class HeartBeat : public rclcpp::Node
{
public:
    HeartBeat() : Node("heart_beat")
    {
        // 【A】订阅 Servo 状态话题
        left_status_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/left_arm_servo_node/status", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                onServoStatus(msg->data, left_servo_status_, left_last_recv_);
            });

        right_status_sub_ = create_subscription<std_msgs::msg::Int8>(
            "/right_arm_servo_node/status", 10,
            [this](const std_msgs::msg::Int8::SharedPtr msg) {
                onServoStatus(msg->data, right_servo_status_, right_last_recv_);
            });

        // 心跳发布者
        publisher_ = create_publisher<planning_sdk_msgs::msg::HeartBeat>(
            "/algorithm/grasp_planning/heart_beat", 10);

        // 1Hz 发布定时器
        publish_timer_ = create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&HeartBeat::publishHeartBeat, this));

        RCLCPP_INFO(get_logger(), "HeartBeat 节点已启动，发布频率 1Hz");
        RCLCPP_INFO(get_logger(), "监控 Servo 状态话题: /left_arm_servo_node/status, /right_arm_servo_node/status");
        RCLCPP_INFO(get_logger(), "监控 Action Server 节点: %zu 个", sizeof(NODE_CHECKS) / sizeof(NODE_CHECKS[0]));
    }

private:
    void onServoStatus(int8_t status, int8_t& stored_status, rclcpp::Time& last_recv)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stored_status = status;
        last_recv = now();
    }

    void publishHeartBeat()
    {
        auto msg = planning_sdk_msgs::msg::HeartBeat();
        auto t = now();

        // ── 【A】Servo 状态 ──────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lock(mutex_);

            // 连接超时检测（曾收到消息后若超时 = 节点异常断联）
            if (isTimedOut(left_last_recv_, t)) {
                addError(msg, 0x0001, "左臂Servo节点无响应");
            }
            if (isTimedOut(right_last_recv_, t)) {
                addError(msg, 0x0002, "右臂Servo节点无响应");
            }

            // Servo 运行状态错误
            collectServoErrors(msg, left_servo_status_,  LEFT_BASE,  "左");
            collectServoErrors(msg, right_servo_status_, RIGHT_BASE, "右");
        }

        // ── 【B】Action Server / Service 节点存活检测 ────────────────────────
        // 使用 count_publishers 检查 Action 内部 status 话题的发布者数量：
        // Action Server 在 spin 期间持续发布该话题，发布者为 0 说明节点未运行。
        for (const auto& check : NODE_CHECKS) {
            if (count_publishers(check.action_status_topic) == 0) {
                addError(msg, check.error_code, check.error_msg);
            }
        }

        publisher_->publish(msg);
    }

    // 曾收到过消息（last_recv 已初始化）且超时，才认定为断联
    // 从未收到（nanoseconds==0）不报警，避免节点启动初期误报
    bool isTimedOut(const rclcpp::Time& last_recv, const rclcpp::Time& t) const
    {
        if (last_recv.nanoseconds() == 0) {
            return false;
        }
        return (t - last_recv).seconds() > SERVO_DISCONNECT_TIMEOUT_SEC;
    }

    // Servo status 码映射为 HeartBeat 错误（base + status_code）
    void collectServoErrors(planning_sdk_msgs::msg::HeartBeat& msg,
                            int8_t status,
                            uint16_t base,
                            const std::string& arm_name)
    {
        if (status == 0) return;

        static const char* const servo_status_text[] = {
            "",                                // 0: 正常
            "Servo: 接近奇异点，正在减速",      // 1
            "Servo: 非常接近奇异点，硬停止",    // 2
            "Servo: 接近碰撞，正在减速",        // 3
            "Servo: 检测到碰撞，硬停止",        // 4
            "Servo: 接近关节限位，停止",         // 5
            "Servo: 正在离开奇异点，减速中",    // 6
        };

        if (status >= 1 && status <= 6) {
            addError(msg,
                static_cast<uint16_t>(base + status),
                arm_name + servo_status_text[status]);
        } else {
            addError(msg,
                static_cast<uint16_t>(base + 0x0F),
                arm_name + "Servo: 未知状态码 " + std::to_string(status));
        }
    }

    void addError(planning_sdk_msgs::msg::HeartBeat& msg,
                  uint16_t code, const std::string& text)
    {
        msg.error_code.push_back(code);
        msg.error_msg.push_back(text);
    }

    std::mutex mutex_;

    int8_t left_servo_status_{0};
    int8_t right_servo_status_{0};

    // nanoseconds()==0 表示从未收到消息
    rclcpp::Time left_last_recv_{0, 0, RCL_ROS_TIME};
    rclcpp::Time right_last_recv_{0, 0, RCL_ROS_TIME};

    rclcpp::Publisher<planning_sdk_msgs::msg::HeartBeat>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr left_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr right_status_sub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeartBeat>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
