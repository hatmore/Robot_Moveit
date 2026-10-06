#include <rclcpp/rclcpp.hpp>
#include <planning_sdk_msgs/srv/register_frame.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class FrameRegistrationClient : public rclcpp::Node
{
public:
    FrameRegistrationClient() : Node("frame_registration_client")
    {
        client_ = this->create_client<planning_sdk_msgs::srv::RegisterFrame>(
            "/algorithm/grasp_planning/register_frame");
    }
    
    bool register_frame(const std::string& parent_frame, 
                       const std::string& child_frame,
                       double x, double y, double z,
                       double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 1.0)
    {
        // 等待服务可用
        if (!client_->wait_for_service(5s)) {
            RCLCPP_ERROR(this->get_logger(), "Service not available");
            return false;
        }
        
        auto request = std::make_shared<planning_sdk_msgs::srv::RegisterFrame::Request>();
        
        // 填充请求数据
        request->timestamp = this->now();
        request->frame.header.stamp = this->now();
        request->frame.header.frame_id = parent_frame;
        request->frame.child_frame_id = child_frame;
        request->frame.transform.translation.x = x;
        request->frame.transform.translation.y = y;
        request->frame.transform.translation.z = z;
        request->frame.transform.rotation.x = qx;
        request->frame.transform.rotation.y = qy;
        request->frame.transform.rotation.z = qz;
        request->frame.transform.rotation.w = qw;
        
        // 发送请求
        auto future = client_->async_send_request(request);
        
        // 等待响应
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            
            auto response = future.get();
            if (response->error_code == 0) {
                RCLCPP_INFO(this->get_logger(), "Frame registration successful");
                return true;
            } else {
                RCLCPP_ERROR(this->get_logger(), "Registration failed with error code: %d", 
                           response->error_code);
                return false;
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed");
            return false;
        }
    }

private:
    rclcpp::Client<planning_sdk_msgs::srv::RegisterFrame>::SharedPtr client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    if (argc < 5) {
        std::cout << "Usage: " << argv[0] << " <parent_frame> <child_frame> <x> <y> <z> [qx qy qz qw]" << std::endl;
        return 1;
    }
    
    auto client = std::make_shared<FrameRegistrationClient>();
    
    std::string parent_frame = argv[1];
    std::string child_frame = argv[2];
    double x = std::stod(argv[3]);
    double y = std::stod(argv[4]);
    double z = std::stod(argv[5]);
    
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    if (argc >= 9) {
        qx = std::stod(argv[6]);
        qy = std::stod(argv[7]);
        qz = std::stod(argv[8]);
        qw = std::stod(argv[9]);
    }
    
    bool success = client->register_frame(parent_frame, child_frame, x, y, z, qx, qy, qz, qw);
    
    rclcpp::shutdown();
    return success ? 0 : 1;
}