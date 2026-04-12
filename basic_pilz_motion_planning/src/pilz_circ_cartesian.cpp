#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <vector>
#include <string>
#include <cmath>
#include <thread>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

const std::string LEFT_ARM_GROUP = "left_arm_group";
const std::string RIGHT_ARM_GROUP = "right_arm_group";

moveit_msgs::msg::Constraints create_circ_path_constraint(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface& move_group,
  const geometry_msgs::msg::Point& mid_point  
) {
  moveit_msgs::msg::Constraints path_constraints;
  moveit_msgs::msg::PositionConstraint pos_constraint;

  pos_constraint.header.frame_id = "base_link";
  pos_constraint.header.stamp = node->now();

  
  pos_constraint.link_name = move_group.getEndEffectorLink();

 
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions = {0.15};  
  pos_constraint.constraint_region.primitives.push_back(sphere);

  
  geometry_msgs::msg::PoseStamped mid_pose_stamped;
  mid_pose_stamped.header = pos_constraint.header;  
  mid_pose_stamped.pose.position = mid_point;      
  mid_pose_stamped.pose.orientation.w = 1.0;        
  pos_constraint.constraint_region.primitive_poses.push_back(mid_pose_stamped.pose);

  
  pos_constraint.weight = 1.0;  

  
  path_constraints.name = "interim"; 
  path_constraints.position_constraints.push_back(pos_constraint);

  return path_constraints;
}

bool move_left_arm_circ_cartesian(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface& left_arm_group,
  const geometry_msgs::msg::PoseStamped& target_pose,
  const geometry_msgs::msg::Point& mid_point
) {
 
  if (target_pose.header.frame_id != "base_link") {
    RCLCPP_ERROR(node->get_logger(), "左臂目标位姿坐标系错误！必须为'base_link'");
    return false;
  }

  auto current_state = left_arm_group.getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(node->get_logger(), "无法获取左臂当前状态！");
    return false;
  }
  std::vector<double> current_velocities;
  current_state->copyJointGroupVelocities(LEFT_ARM_GROUP, current_velocities);
  for (double vel : current_velocities) {
    if (std::fabs(vel) > 1e-6) {
      RCLCPP_ERROR(node->get_logger(), "左臂起始速度非零！CIRC规划要求静止启动");
      return false;
    }
  }

  left_arm_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  left_arm_group.setPlannerId("CIRC");
 
  RCLCPP_INFO(node->get_logger(), "左臂规划管道: %s", left_arm_group.getPlanningPipelineId().c_str());
  RCLCPP_INFO(node->get_logger(), "左臂规划器ID: %s", left_arm_group.getPlannerId().c_str());

  left_arm_group.setPoseTarget(target_pose);

  left_arm_group.setMaxVelocityScalingFactor(0.1);
  left_arm_group.setMaxAccelerationScalingFactor(0.1);
  left_arm_group.setPlanningTime(20.0);  

  auto circ_constraint = create_circ_path_constraint(node, left_arm_group, mid_point);
  left_arm_group.setPathConstraints(circ_constraint);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (left_arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (plan_success) {
    RCLCPP_INFO(node->get_logger(), "左臂CIRC圆弧规划成功，执行运动...");
    bool exec_success = (left_arm_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (exec_success) {
      RCLCPP_INFO(node->get_logger(), "左臂CIRC圆弧运动完成");
      return true;
    } else {
      RCLCPP_ERROR(node->get_logger(), "左臂运动执行失败");
      return false;
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "左臂CIRC规划失败！可能原因：三点共线、超出工作空间、IK无解");
    return false;
  }
}

bool move_right_arm_circ_cartesian(
  rclcpp::Node::SharedPtr node,
  moveit::planning_interface::MoveGroupInterface& right_arm_group,
  const geometry_msgs::msg::PoseStamped& target_pose,
  const geometry_msgs::msg::Point& mid_point
) {
  
  if (target_pose.header.frame_id != "base_link") {
    RCLCPP_ERROR(node->get_logger(), "右臂目标位姿坐标系错误！必须为'base_link'");
    return false;
  }


  auto current_state = right_arm_group.getCurrentState();
  if (!current_state) {
    RCLCPP_ERROR(node->get_logger(), "无法获取右臂当前状态！");
    return false;
  }
  std::vector<double> current_velocities;
  current_state->copyJointGroupVelocities(RIGHT_ARM_GROUP, current_velocities);
  for (double vel : current_velocities) {
    if (std::fabs(vel) > 1e-6) {
      RCLCPP_ERROR(node->get_logger(), "右臂起始速度非零！CIRC规划要求静止启动");
      return false;
    }
  }

  right_arm_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  right_arm_group.setPlannerId("CIRC");
  

  RCLCPP_INFO(node->get_logger(), "右臂规划管道: %s", right_arm_group.getPlanningPipelineId().c_str());
  RCLCPP_INFO(node->get_logger(), "右臂规划器ID: %s", right_arm_group.getPlannerId().c_str());

  right_arm_group.setPoseTarget(target_pose);

  right_arm_group.setMaxVelocityScalingFactor(0.1);
  right_arm_group.setMaxAccelerationScalingFactor(0.1);
  right_arm_group.setPlanningTime(20.0);

  auto circ_constraint = create_circ_path_constraint(node, right_arm_group, mid_point);
  right_arm_group.setPathConstraints(circ_constraint);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool plan_success = (right_arm_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (plan_success) {
    RCLCPP_INFO(node->get_logger(), "右臂CIRC圆弧规划成功，执行运动...");
    bool exec_success = (right_arm_group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (exec_success) {
      RCLCPP_INFO(node->get_logger(), "右臂CIRC圆弧运动完成");
      return true;
    } else {
      RCLCPP_ERROR(node->get_logger(), "右臂运动执行失败");
      return false;
    }
  } else {
    RCLCPP_ERROR(node->get_logger(), "右臂CIRC规划失败！");
    return false;
  }
}

void run_executor(rclcpp::executors::MultiThreadedExecutor::SharedPtr executor, rclcpp::Node::SharedPtr node) {
  RCLCPP_INFO(node->get_logger(), "多线程执行器启动，处理回调...");
  executor->spin();
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pilz_circ_cartesian_demo");
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  executor->add_node(node);
  std::thread executor_thread(run_executor, executor, node);
  rclcpp::sleep_for(std::chrono::seconds(1));  // 等待执行器初始化

  RCLCPP_INFO(node->get_logger(), "Pilz CIRC（笛卡尔空间目标） Demo 启动");

  moveit::planning_interface::MoveGroupInterface left_arm_group(node, LEFT_ARM_GROUP);
  moveit::planning_interface::MoveGroupInterface right_arm_group(node, RIGHT_ARM_GROUP);

  auto left_current_pose = left_arm_group.getCurrentPose();
  auto right_current_pose = right_arm_group.getCurrentPose();
  RCLCPP_INFO(node->get_logger(), "左臂当前位置：x=%.3f, y=%.3f, z=%.3f",
              left_current_pose.pose.position.x,
              left_current_pose.pose.position.y,
              left_current_pose.pose.position.z);
  RCLCPP_INFO(node->get_logger(), "右臂当前位置：x=%.3f, y=%.3f, z=%.3f",
              right_current_pose.pose.position.x,
              right_current_pose.pose.position.y,
              right_current_pose.pose.position.z);

  geometry_msgs::msg::PoseStamped left_target_pose;
  left_target_pose.header.frame_id = "base_link";
  left_target_pose.header.stamp = node->now();
  left_target_pose.pose.position.x = left_current_pose.pose.position.x + 0.1;  
  left_target_pose.pose.position.y = left_current_pose.pose.position.y - 0.1;  
  left_target_pose.pose.position.z = left_current_pose.pose.position.z;       
  left_target_pose.pose.orientation = left_current_pose.pose.orientation;    

  geometry_msgs::msg::Point left_mid_point;
  left_mid_point.x = (left_current_pose.pose.position.x + left_target_pose.pose.position.x) / 2;
  left_mid_point.y = (left_current_pose.pose.position.y + left_target_pose.pose.position.y) / 2;
  left_mid_point.z = left_current_pose.pose.position.z + 0.05; 

  geometry_msgs::msg::PoseStamped right_target_pose;
  right_target_pose.header.frame_id = "base_link";
  right_target_pose.header.stamp = node->now();
  right_target_pose.pose.position.x = right_current_pose.pose.position.x + 0.1; 
  right_target_pose.pose.position.y = right_current_pose.pose.position.y + 0.1; 
  right_target_pose.pose.position.z = right_current_pose.pose.position.z;        
  right_target_pose.pose.orientation = right_current_pose.pose.orientation;      

  geometry_msgs::msg::Point right_mid_point;
  right_mid_point.x = (right_current_pose.pose.position.x + right_target_pose.pose.position.x) / 2;
  right_mid_point.y = (right_current_pose.pose.position.y + right_target_pose.pose.position.y) / 2;
  right_mid_point.z = right_current_pose.pose.position.z + 0.05; 

  move_left_arm_circ_cartesian(node, left_arm_group, left_target_pose, left_mid_point);
  rclcpp::sleep_for(std::chrono::seconds(2)); 
  move_right_arm_circ_cartesian(node, right_arm_group, right_target_pose, right_mid_point);

  executor->cancel();
  executor->remove_node(node);
  executor_thread.join();

  RCLCPP_INFO(node->get_logger(), "Demo 结束");
  rclcpp::shutdown();
  return 0;
}