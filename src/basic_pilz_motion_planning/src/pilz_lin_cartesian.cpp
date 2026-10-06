#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/orientation_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <eigen3/Eigen/Dense>

const std::string LEFT_END_EFFECTOR = "left_vacuum_base_link";
const std::string RIGHT_END_EFFECTOR = "right_vacuum_base_link";
const std::string BASE_FRAME = "base_link";
const std::string LEFT_GROUP = "left_arm_group";
const std::string RIGHT_GROUP = "right_arm_group";

// bool check_zero_velocity(const moveit::planning_interface::MoveGroupInterface& move_group)
// {
//   const moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);
//   if (!current_state) {
//     RCLCPP_ERROR(rclcpp::get_logger("zero_velocity_check"), "无法获取当前机器人状态（超时）");
//     return false;
//   }

//   const std::vector<std::string>& joint_names = move_group.getJoints();
//   const double* velocity_ptr = current_state->getVariableVelocities();
//   Eigen::Map<const Eigen::VectorXd> velocities(velocity_ptr, joint_names.size());

//   const double velocity_threshold = 1e-4;
//   for (size_t i = 0; i < joint_names.size(); ++i) {
//     if (std::abs(velocities[i]) > velocity_threshold) {
//       RCLCPP_ERROR(rclcpp::get_logger("zero_velocity_check"),
//                   "关节 [%s] 起始速度不为零 (%.6f rad/s)，LIN规划失败",
//                   joint_names[i].c_str(), velocities[i]);
//       return false;
//     }
//   }

//   RCLCPP_INFO(rclcpp::get_logger("zero_velocity_check"), "起始关节速度检查通过（均为零）");
//   return true;
// }

moveit_msgs::msg::Constraints create_cartesian_constraints(
  const geometry_msgs::msg::Pose& target_pose,
  const std::string& end_effector_link,
  double position_tolerance,
  double orientation_tolerance
)
{
  moveit_msgs::msg::Constraints constraints;

  moveit_msgs::msg::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = BASE_FRAME;
  pos_constraint.link_name = end_effector_link;
  pos_constraint.constraint_region.primitive_poses.push_back(target_pose);
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions = {position_tolerance};
  pos_constraint.constraint_region.primitives.push_back(sphere);
  constraints.position_constraints.push_back(pos_constraint);

  moveit_msgs::msg::OrientationConstraint orient_constraint;
  orient_constraint.header.frame_id = BASE_FRAME;
  orient_constraint.link_name = end_effector_link;
  orient_constraint.orientation = target_pose.orientation;
  orient_constraint.absolute_x_axis_tolerance = orientation_tolerance;
  orient_constraint.absolute_y_axis_tolerance = orientation_tolerance;
  orient_constraint.absolute_z_axis_tolerance = orientation_tolerance;
  orient_constraint.weight = 1.0;
  constraints.orientation_constraints.push_back(orient_constraint);

  return constraints;
}

bool move_arm_lin_cartesian(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface& move_group,
  const geometry_msgs::msg::Pose& target_pose,
  const std::string& end_effector_link
)
{
  move_group.setEndEffectorLink(end_effector_link);
  move_group.setPoseReferenceFrame(BASE_FRAME);

  move_group.setPlannerId("LIN");
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setPlanningTime(10.0);

  move_group.setPoseTarget(target_pose);
  const double pos_tol = 0.001;
  const double orient_tol = 0.01;
  move_group.setGoalPositionTolerance(pos_tol);
  move_group.setGoalOrientationTolerance(orient_tol);

//   if (!check_zero_velocity(move_group)) {
//     return false;
//   }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (!plan_success) {
    RCLCPP_ERROR(node->get_logger(), "LIN规划失败！可能原因：位姿不可达、路径碰撞、奇异点");
    return false;
  }

  RCLCPP_INFO(node->get_logger(), "LIN规划成功，开始执行（轨迹包含 %zu 个路点）",
             plan.trajectory_.joint_trajectory.points.size());
  bool exec_success = (move_group.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (exec_success) {
    RCLCPP_INFO(node->get_logger(), "LIN运动执行完成");
    return true;
  } else {
    RCLCPP_ERROR(node->get_logger(), "LIN运动执行失败");
    return false;
  }
}

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pilz_lin_cartesian_full_demo");
  RCLCPP_INFO(node->get_logger(), "Pilz LIN笛卡尔空间规划演示启动");

  moveit::planning_interface::MoveGroupInterface left_arm(node, LEFT_GROUP);
  moveit::planning_interface::MoveGroupInterface right_arm(node, RIGHT_GROUP);

  left_arm.setPlanningPipelineId("pilz_industrial_motion_planner");
  right_arm.setPlanningPipelineId("pilz_industrial_motion_planner");

  geometry_msgs::msg::Pose left_target;
  left_target.position.x = 0.6;
  left_target.position.y = 0.4;
  left_target.position.z = 0.7;
  tf2::Quaternion left_quat;
  left_quat.setRPY(0.0, M_PI/2, 0.0);
  left_target.orientation.x = left_quat.x();
  left_target.orientation.y = left_quat.y();
  left_target.orientation.z = left_quat.z();
  left_target.orientation.w = left_quat.w();

  geometry_msgs::msg::Pose right_target;
  right_target.position.x = 0.6;
  right_target.position.y = -0.4;
  right_target.position.z = 0.7;
  tf2::Quaternion right_quat;
  right_quat.setRPY(0.0, M_PI/2, 0.0);
  right_target.orientation.x = right_quat.x();
  right_target.orientation.y = right_quat.y();
  right_target.orientation.z = right_quat.z();
  right_target.orientation.w = right_quat.w();

  RCLCPP_INFO(node->get_logger(), "开始执行左臂LIN运动...");
  move_arm_lin_cartesian(node, left_arm, left_target, LEFT_END_EFFECTOR);

  rclcpp::sleep_for(std::chrono::seconds(1));

  RCLCPP_INFO(node->get_logger(), "开始执行右臂LIN运动...");
  move_arm_lin_cartesian(node, right_arm, right_target, RIGHT_END_EFFECTOR);

  RCLCPP_INFO(node->get_logger(), "演示结束，关闭节点");
  rclcpp::shutdown();
  return 0;
}
