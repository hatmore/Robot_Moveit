#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>  

bool move_left_arm_ptp(rclcpp::Node::SharedPtr node, 
                      moveit::planning_interface::MoveGroupInterface& left_arm_group,
                      const geometry_msgs::msg::Pose& target_pose)
{
    left_arm_group.setPoseTarget(target_pose);

    left_arm_group.setPlannerId("PTP");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (left_arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "左臂PTP规划成功，执行运动...");
        left_arm_group.execute(plan);  
        return true;
    } else {
        RCLCPP_ERROR(node->get_logger(), "左臂PTP规划失败！");
        return false;
    }
}

bool move_right_arm_ptp(rclcpp::Node::SharedPtr node, 
                       moveit::planning_interface::MoveGroupInterface& right_arm_group,
                       const geometry_msgs::msg::Pose& target_pose)
{
    right_arm_group.setPoseTarget(target_pose);

    right_arm_group.setPlannerId("PTP");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (right_arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        RCLCPP_INFO(node->get_logger(), "右臂PTP规划成功，执行运动...");
        right_arm_group.execute(plan);
        return true;
    } else {
        RCLCPP_ERROR(node->get_logger(), "右臂PTP规划失败！");
        return false;
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pilz_cartesian_space_ptp_demo");

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> left_arm_group =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "left_arm_group");
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> right_arm_group =
        std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "right_arm_group");

    left_arm_group->setPlanningPipelineId("pilz_industrial_motion_planner");
    right_arm_group->setPlanningPipelineId("pilz_industrial_motion_planner");

    left_arm_group->setPlanningTime(5.0);
    right_arm_group->setPlanningTime(5.0);

    left_arm_group->setGoalPositionTolerance(0.01);
    left_arm_group->setGoalOrientationTolerance(0.01);
    right_arm_group->setGoalPositionTolerance(0.01);
    right_arm_group->setGoalOrientationTolerance(0.01);

    RCLCPP_INFO(node->get_logger(), "左臂末端执行器: %s", 
                left_arm_group->getEndEffectorLink().c_str());
    RCLCPP_INFO(node->get_logger(), "右臂末端执行器: %s", 
                right_arm_group->getEndEffectorLink().c_str());

    geometry_msgs::msg::Pose left_target_pose;
    left_target_pose.position.x = 0.2;  
    left_target_pose.position.y = 0.2;  
    left_target_pose.position.z = 0.4;  

    tf2::Quaternion left_quat;
    left_quat.setRPY(0.0, M_PI / 2, 0.0);
    left_target_pose.orientation.x = left_quat.x();
    left_target_pose.orientation.y = left_quat.y();
    left_target_pose.orientation.z = left_quat.z();
    left_target_pose.orientation.w = left_quat.w();

    geometry_msgs::msg::Pose right_target_pose;
    right_target_pose.position.x = 0.2;
    right_target_pose.position.y = -0.2;  
    right_target_pose.position.z = 0.4;
    
    tf2::Quaternion right_quat;
    right_quat.setRPY(0.0, M_PI / 2, 0.0);
    right_target_pose.orientation.x = right_quat.x();
    right_target_pose.orientation.y = right_quat.y();
    right_target_pose.orientation.z = right_quat.z();
    right_target_pose.orientation.w = right_quat.w();

    move_left_arm_ptp(node, *left_arm_group, left_target_pose);
    
    rclcpp::sleep_for(std::chrono::seconds(1));
    move_right_arm_ptp(node, *right_arm_group, right_target_pose);

    left_arm_group.reset();
    right_arm_group.reset();

    rclcpp::shutdown();
    return 0;
}
