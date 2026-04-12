#include <rclcpp/rclcpp.hpp>
#include <planning_sdk_msgs/srv/register_frame.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <memory>

class FrameRegistrationServer : public rclcpp::Node
{
public:
    FrameRegistrationServer() : Node("frame_registration_server")
    {
        service_ = this->create_service<planning_sdk_msgs::srv::RegisterFrame>(
            "/algorithm/grasp_planning/register_frame",
            std::bind(&FrameRegistrationServer::handle_request, this, 
                     std::placeholders::_1, std::placeholders::_2));
        
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        RCLCPP_INFO(this->get_logger(), "Frame registration server ready");
    }

private:
    void handle_request(
        const std::shared_ptr<planning_sdk_msgs::srv::RegisterFrame::Request> request,
        std::shared_ptr<planning_sdk_msgs::srv::RegisterFrame::Response> response)
    {
        // 设置响应时间戳
        response->timestamp = this->now();
        
        try {
            // 验证请求
            if (request->frame.header.frame_id.empty()) {
                response->error_code = 400;
                RCLCPP_ERROR(this->get_logger(), "Parent frame ID is empty");
                return;
            }
            
            if (request->frame.child_frame_id.empty()) {
                response->error_code = 401;
                RCLCPP_ERROR(this->get_logger(), "Child frame ID is empty");
                return;
            }
            
            // 发布静态TF变换
            std::vector<geometry_msgs::msg::TransformStamped> transforms = {request->frame};
            tf_broadcaster_->sendTransform(transforms);
            
            response->error_code = 0; // 成功
            RCLCPP_INFO(this->get_logger(), "Registered frame: %s -> %s", 
                       request->frame.header.frame_id.c_str(),
                       request->frame.child_frame_id.c_str());
            
        } catch (const std::exception& e) {
            response->error_code = 500;
            RCLCPP_ERROR(this->get_logger(), "Registration failed: %s", e.what());
        }
    }
    
    rclcpp::Service<planning_sdk_msgs::srv::RegisterFrame>::SharedPtr service_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<FrameRegistrationServer>();
    rclcpp::spin(server);
    rclcpp::shutdown();
    return 0;
}