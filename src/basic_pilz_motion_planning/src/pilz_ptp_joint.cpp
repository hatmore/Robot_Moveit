#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

const std::vector<std::string> LEFT_ARM_JOINT_NAMES = {
  "left_shoulder_pitch_joint",
  "left_shoulder_roll_joint",
  "left_shoulder_yaw_joint",
  "left_elbow_joint",
  "left_wrist_roll_joint",
  "left_wrist_yaw_joint",
  "left_wrist_pitch_joint"
};

const std::vector<std::string> RIGHT_ARM_JOINT_NAMES = {
  "right_shoulder_pitch_joint",
  "right_shoulder_roll_joint",
  "right_shoulder_yaw_joint",
  "right_elbow_joint",
  "right_wrist_roll_joint",
  "right_wrist_yaw_joint",
  "right_wrist_pitch_joint"
};

bool move_left_arm_ptp(rclcpp::Node::SharedPtr node, 
                      moveit::planning_interface::MoveGroupInterface& left_arm_group,
                      const std::vector<double>& target_joint_values)
{
  if (target_joint_values.size() != LEFT_ARM_JOINT_NAMES.size()) {
    RCLCPP_ERROR(node->get_logger(), "左臂目标关节数量不匹配！期望 %zu 个，实际 %zu 个",
                 LEFT_ARM_JOINT_NAMES.size(), target_joint_values.size());
    return false;
  }

  std::map<std::string, double> joint_target;
  for (size_t i = 0; i < LEFT_ARM_JOINT_NAMES.size(); ++i) {
    joint_target[LEFT_ARM_JOINT_NAMES[i]] = target_joint_values[i];
  }
  left_arm_group.setJointValueTarget(joint_target);

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
                       const std::vector<double>& target_joint_values)
{
  if (target_joint_values.size() != RIGHT_ARM_JOINT_NAMES.size()) {
    RCLCPP_ERROR(node->get_logger(), "右臂目标关节数量不匹配！期望 %zu 个，实际 %zu 个",
                 RIGHT_ARM_JOINT_NAMES.size(), target_joint_values.size());
    return false;
  }

  std::map<std::string, double> joint_target;
  for (size_t i = 0; i < RIGHT_ARM_JOINT_NAMES.size(); ++i) {
    joint_target[RIGHT_ARM_JOINT_NAMES[i]] = target_joint_values[i];
  }
  right_arm_group.setJointValueTarget(joint_target);

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
  auto node = rclcpp::Node::make_shared("pilz_joint_space_ptp_demo");

  moveit::planning_interface::MoveGroupInterface left_arm_group(node, "left_arm_group");
  moveit::planning_interface::MoveGroupInterface right_arm_group(node, "right_arm_group");

  left_arm_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  right_arm_group.setPlanningPipelineId("pilz_industrial_motion_planner");

  left_arm_group.setPlanningTime(5.0);
  right_arm_group.setPlanningTime(5.0);
  left_arm_group.setGoalJointTolerance(0.01);
  right_arm_group.setGoalJointTolerance(0.01);

  std::vector<double> left_target_joints = {
    0.0,
    0.5,
    0.0,
    -1.0,
    0.0,
    0.0,
    0.0
  };

  std::vector<double> right_target_joints = {
    0.0,
    -0.5,
    0.0,
    -1.0,
    0.0,
    0.0,
    0.0
  };

  move_left_arm_ptp(node, left_arm_group, left_target_joints);

  rclcpp::sleep_for(std::chrono::seconds(1));
  move_right_arm_ptp(node, right_arm_group, right_target_joints);

  rclcpp::shutdown();
  return 0;
}
