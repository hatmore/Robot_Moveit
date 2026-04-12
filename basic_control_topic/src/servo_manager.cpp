#include "rclcpp/rclcpp.hpp"
#include <planning_sdk_msgs/msg/joint_velocity_once.hpp>
#include <planning_sdk_msgs/msg/tcp_velocity_once.hpp>
#include "std_srvs/srv/trigger.hpp"

#include <mutex>
#include <thread>

using Trigger = std_srvs::srv::Trigger;

class ServoManager : public rclcpp::Node
{
public:
    ServoManager() : Node("servo_manager")
    {
        servo_joint_sub_ = this->create_subscription<planning_sdk_msgs::msg::JointVelocityOnce>(
            "/algorithm/grasp_planning/move/joint_velocity_once",
            10,
            std::bind(&ServoManager::jointCallback, this, std::placeholders::_1));
            
        servo_left_tcp_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
            "/algorithm/grasp_planning/move/left_arm/tcp_velocity_once",
            10,
            std::bind(&ServoManager::leftTcpCallback, this, std::placeholders::_1));
                
        servo_right_tcp_sub_ = this->create_subscription<planning_sdk_msgs::msg::TcpVelocityOnce>(
            "/algorithm/grasp_planning/move/right_arm/tcp_velocity_once",
            10,
            std::bind(&ServoManager::rightTcpCallback, this, std::placeholders::_1));
                    
        spin_node_ = std::make_shared<rclcpp::Node>("servo_manager_spin_node");
        left_start_cli_ = spin_node_->create_client<Trigger>("/left_arm_servo_node/start_servo");
        left_stop_cli_ = spin_node_->create_client<Trigger>("/left_arm_servo_node/stop_servo");

        right_start_cli_ = spin_node_->create_client<Trigger>("/right_arm_servo_node/start_servo");
        right_stop_cli_ = spin_node_->create_client<Trigger>("/right_arm_servo_node/stop_servo");

        check_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ServoManager::checkCallback, this));

        is_servo_running_ = false;
    }

private:
    void jointCallback(const planning_sdk_msgs::msg::JointVelocityOnce::SharedPtr msg)
    {
        updateLastTime();
        requestStartServo();
    }

    void leftTcpCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg)
    {
        updateLastTime();
        requestStartServo();
    }

    void rightTcpCallback(const planning_sdk_msgs::msg::TcpVelocityOnce::SharedPtr msg)
    {
        updateLastTime();
        requestStartServo();
    }

    void requestStartServo()
    {
        if (is_servo_running_ || is_starting_) return;
        is_starting_ = true;
        std::thread([this]() {
            startServo();
            is_starting_ = false;
        }).detach();
    }

    void updateLastTime()
    {
        std::lock_guard<std::mutex> lock(last_time_mutex_);
        last_time_ = this->get_clock()->now();
    }

    void checkCallback()
    {
        if (is_servo_running_)
        {
            std::lock_guard<std::mutex> lock(last_time_mutex_);
            if ((this->get_clock()->now() - last_time_).nanoseconds() > 1000000000)
            {
                std::thread([this]() { stopServo(); }).detach();
            }
        }
    }

    void startServo()
    {
        if (!left_start_cli_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "Left arm servo start service not available");
            return;
        }

        if (!right_start_cli_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "Right arm servo start service not available");
            return;
        }

        // 左右臂并行发送启动请求
        auto left_future = left_start_cli_->async_send_request(std::make_shared<Trigger::Request>());
        auto right_future = right_start_cli_->async_send_request(std::make_shared<Trigger::Request>());

        bool left_ok = false, right_ok = false;

        if (rclcpp::spin_until_future_complete(spin_node_, left_future, std::chrono::seconds(3)) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto resp = left_future.get();
            left_ok = resp->success;
            if (!left_ok) {
                RCLCPP_ERROR(this->get_logger(), "Left arm servo start failed: %s", resp->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Left arm servo start service call timeout");
        }

        if (rclcpp::spin_until_future_complete(spin_node_, right_future, std::chrono::seconds(3)) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            auto resp = right_future.get();
            right_ok = resp->success;
            if (!right_ok) {
                RCLCPP_ERROR(this->get_logger(), "Right arm servo start failed: %s", resp->message.c_str());
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Right arm servo start service call timeout");
        }

        if (left_ok && right_ok) {
            RCLCPP_INFO(this->get_logger(), "servo auto start !!");
            is_servo_running_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "servo start partial: left=%s, right=%s",
                        left_ok ? "ok" : "failed", right_ok ? "ok" : "failed");
            // 即使只有一个臂启动成功也标记为运行，避免反复重试阻塞
            is_servo_running_ = true;
        }
    }

    void stopServo()
    {
        // 不阻塞等待 service 可用，直接尝试发送
        if (left_stop_cli_->service_is_ready()) {
            left_stop_cli_->async_send_request(std::make_shared<Trigger::Request>());
        }
        if (right_stop_cli_->service_is_ready()) {
            right_stop_cli_->async_send_request(std::make_shared<Trigger::Request>());
        }

        RCLCPP_INFO(this->get_logger(), "servo auto stop !!");
        is_servo_running_ = false;
    }

private:
    rclcpp::Client<Trigger>::SharedPtr left_start_cli_;
    rclcpp::Client<Trigger>::SharedPtr left_stop_cli_;

    rclcpp::Client<Trigger>::SharedPtr right_start_cli_;
    rclcpp::Client<Trigger>::SharedPtr right_stop_cli_;

    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr servo_left_tcp_sub_;
    rclcpp::Subscription<planning_sdk_msgs::msg::TcpVelocityOnce>::SharedPtr servo_right_tcp_sub_;

    rclcpp::Subscription<planning_sdk_msgs::msg::JointVelocityOnce>::SharedPtr servo_joint_sub_;

    rclcpp::TimerBase::SharedPtr check_timer_;

    std::mutex last_time_mutex_;
    rclcpp::Time last_time_;

    std::atomic<bool> is_servo_running_;
    std::atomic<bool> is_starting_{false};

    rclcpp::Node::SharedPtr spin_node_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServoManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}