// initial_joint_positions.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <chrono>
#include <thread>

class InitialJointPositions : public rclcpp::Node
{
public:
    InitialJointPositions() : Node("initial_joint_positions")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/initial_joint_states", 10);
        
        // 发布初始位置
        auto timer_callback = [this]() -> void {
            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->now();
            message.header.frame_id = "base_link";
            
            // 设置关节名称
            message.name = {
                "left_shoulder_pitch_joint",
                "left_shoulder_roll_joint", 
                "left_shoulder_yaw_joint",
                "left_elbow_joint",
                "left_wrist_roll_joint",
                "left_wrist_yaw_joint",
                "left_wrist_pitch_joint"
            };
            
            // 设置初始位置（弧度）
            message.position = {
                0.5,   // left_shoulder_pitch_joint
                0.2,   // left_shoulder_roll_joint
                0.0,   // left_shoulder_yaw_joint  
                -0.3,  // left_elbow_joint
                0.1,   // left_wrist_roll_joint
                0.0,   // left_wrist_yaw_joint
                0.2    // left_wrist_pitch_joint
            };
            
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published initial joint positions");
            
            // 发布几次后停止
            if (++publish_count_ >= 5) {
                timer_->cancel();
                rclcpp::shutdown();
            }
        };
        
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int publish_count_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitialJointPositions>();
    rclcpp::spin(node);
    return 0;
}