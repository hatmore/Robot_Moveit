#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "geometry_msgs/msg/pose.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "planning_sdk_msgs/msg/tcp_pose.hpp"

class TcpPosePublisher : public rclcpp::Node {
public:
  TcpPosePublisher() : Node("tcp_pose_publisher") {
    this->declare_parameter<std::string>("left_source_frame", "left_gripper_base_link");
    this->declare_parameter<std::string>("right_source_frame", "right_gripper_base_link");
    this->declare_parameter<std::string>("target_frame", "base_link");

    left_source_frame_ = this->get_parameter("left_source_frame").as_string();
    right_source_frame_ = this->get_parameter("right_source_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();

    left_publisher_ = this->create_publisher<planning_sdk_msgs::msg::TcpPose>(
      "/algorithm/grasp_planning/left_arm/tcp_pose", 10);
    right_publisher_ = this->create_publisher<planning_sdk_msgs::msg::TcpPose>(
      "/algorithm/grasp_planning/right_arm/tcp_pose", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TcpPosePublisher::publish_tcp_pose, this));

    RCLCPP_INFO(this->get_logger(), "TcpPose发布节点已启动！");
    RCLCPP_INFO(this->get_logger(), "当前配置：");
    RCLCPP_INFO(this->get_logger(), "  目标坐标系：%s", target_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  左臂源帧：%s", left_source_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  右臂源帧：%s", right_source_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "  发布频率：1.00 Hz");
    RCLCPP_INFO(this->get_logger(), "  左臂话题：/algorithm/grasp_planning/left_arm/tcp_pose");
    RCLCPP_INFO(this->get_logger(), "  右臂话题：/algorithm/grasp_planning/right_arm/tcp_pose");
  }

private:
  void get_and_publish_tcp_pose(
    const std::string& source_frame,
    rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr publisher,
    const std::string& arm_name) {

    auto msg = planning_sdk_msgs::msg::TcpPose();
    msg.error_code = 0;
    msg.timestamp = this->get_clock()->now();

    try {
      auto transform = tf_buffer_->lookupTransform(
        target_frame_,
        source_frame,
        tf2::TimePointZero,
        std::chrono::milliseconds(500)
      );

      geometry_msgs::msg::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation = transform.transform.rotation;
      msg.current_pose = pose;

      RCLCPP_DEBUG(this->get_logger(), "%s TF查询成功：%s → %s", 
                   arm_name.c_str(), source_frame.c_str(), target_frame_.c_str());

    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "%s TF查询失败（%s → %s）：%s", 
                  arm_name.c_str(), source_frame.c_str(), target_frame_.c_str(), ex.what());
      msg.error_code = 1;
    }

    publisher->publish(msg);
    //RCLCPP_DEBUG(this->get_logger(), "已发布%sTcpPose，错误码：%d", arm_name.c_str(), msg.error_code);
  }

  void publish_tcp_pose() {
    get_and_publish_tcp_pose(left_source_frame_, left_publisher_, "左臂");
    get_and_publish_tcp_pose(right_source_frame_, right_publisher_, "右臂");
  }

  rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr left_publisher_;
  rclcpp::Publisher<planning_sdk_msgs::msg::TcpPose>::SharedPtr right_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string left_source_frame_;
  std::string right_source_frame_;
  std::string target_frame_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
