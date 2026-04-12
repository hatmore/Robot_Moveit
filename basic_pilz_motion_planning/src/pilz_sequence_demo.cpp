#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/planning_options.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_motion_sequence.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>

/**
 * 适配左臂（left_arm_group）的Pilz序列运动程序（PTP→LIN→CIRC）
 */

using moveit_msgs::action::MoveGroupSequence;
using GoalHandleMoveGroupSequence = rclcpp_action::ClientGoalHandle<MoveGroupSequence>;

auto const LOGGER = rclcpp::get_logger("pilz_left_arm_sequence_node");

int main(int argc, char**argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("pilz_left_arm_sequence_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "left_arm_group";
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
  std::string end_effector_link = "left_vacuum_plane_link";

  auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{ node, "base_link", "left_arm_move_group_tutorial",
                                                                     move_group_interface.getRobotModel() };
  moveit_visual_tools.deleteAllMarkers();
  moveit_visual_tools.loadRemoteControl();
  moveit_visual_tools.prompt("Press 'next' to start the demo");

  // ----- 第一段：PTP运动
  moveit_msgs::msg::MotionSequenceItem item1;
  item1.blend_radius = 0.1;
  item1.req.group_name = PLANNING_GROUP;
  item1.req.planner_id = "PTP";
  item1.req.allowed_planning_time = 5.0;
  item1.req.max_velocity_scaling_factor = 0.1;
  item1.req.max_acceleration_scaling_factor = 0.1;

  auto target_pose_item1 = [] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position.x = 0.2;
    msg.pose.position.y = 0.2;
    msg.pose.position.z = 0.4;
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  item1.req.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints(end_effector_link, target_pose_item1)
  );

  // ----- 第二段：LIN运动
  moveit_msgs::msg::MotionSequenceItem item2;
  item2.blend_radius = 0.1;
  item2.req.group_name = PLANNING_GROUP;
  item2.req.planner_id = "LIN";
  item2.req.allowed_planning_time = 5.0;
  item2.req.max_velocity_scaling_factor = 0.1;
  item2.req.max_acceleration_scaling_factor = 0.1;

  auto target_pose_item2 = [] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position.x = 0.35;  
    msg.pose.position.y = -0.25; 
    msg.pose.position.z = 0.65; 
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  item2.req.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints(end_effector_link, target_pose_item2)
  );

  // ----- 第三段：CIRC运动
  moveit_msgs::msg::MotionSequenceItem item3;
  item3.blend_radius = 0.0;
  item3.req.group_name = PLANNING_GROUP;
  item3.req.planner_id = "CIRC";
  item3.req.allowed_planning_time = 15.0;
  item3.req.max_velocity_scaling_factor = 0.08;
  item3.req.max_acceleration_scaling_factor = 0.08;

  auto target_pose_item3 = [] {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.pose.position.x = 0.5;  
    msg.pose.position.y = -0.15; 
    msg.pose.position.z = 0.6;   
    msg.pose.orientation.x = 1.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 0.0;
    return msg;
  }();
  item3.req.goal_constraints.push_back(
    kinematic_constraints::constructGoalConstraints(end_effector_link, target_pose_item3)
  );

  
  geometry_msgs::msg::Point circ_mid_point;
  circ_mid_point.x = 0.425; 
  circ_mid_point.y = -0.2;  
  circ_mid_point.z = 0.63;   

  
  RCLCPP_INFO(LOGGER, "CIRC运动三点坐标:");
  RCLCPP_INFO(LOGGER, "起点(第二段终点): (%.3f, %.3f, %.3f)",
              0.35, -0.25, 0.65);
  RCLCPP_INFO(LOGGER, "中间点: (%.3f, %.3f, %.3f)",
              circ_mid_point.x, circ_mid_point.y, circ_mid_point.z);
  RCLCPP_INFO(LOGGER, "终点: (%.3f, %.3f, %.3f)",
              0.5, -0.15, 0.6);

  
  moveit_msgs::msg::Constraints circ_constraint;
  moveit_msgs::msg::PositionConstraint pos_constraint;
  pos_constraint.header.frame_id = "world";
  pos_constraint.link_name = end_effector_link;
  
  shape_msgs::msg::SolidPrimitive sphere;
  sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
  sphere.dimensions = {0.2};
  pos_constraint.constraint_region.primitives.push_back(sphere);
  
  geometry_msgs::msg::Pose mid_pose;
  mid_pose.position = circ_mid_point;
  mid_pose.orientation.w = 1.0;
  pos_constraint.constraint_region.primitive_poses.push_back(mid_pose);
  
  pos_constraint.weight = 1.0;
  circ_constraint.position_constraints.push_back(pos_constraint);
  circ_constraint.name = "interim";
  item3.req.path_constraints = circ_constraint;

  using GetMotionSequence = moveit_msgs::srv::GetMotionSequence;
  auto service_client = node->create_client<GetMotionSequence>("/plan_sequence_path");

  while (!service_client->wait_for_service(std::chrono::seconds(10))) {
    RCLCPP_WARN(LOGGER, "Waiting for service /plan_sequence_path...");
  }

  auto service_request = std::make_shared<GetMotionSequence::Request>();
  service_request->request.items.push_back(item1);
  service_request->request.items.push_back(item2);
  service_request->request.items.push_back(item3);

  auto service_future = service_client->async_send_request(service_request);

  auto const draw_trajectory_tool_path =
      [&moveit_visual_tools,
       jmg = move_group_interface.getRobotModel()->getJointModelGroup(PLANNING_GROUP)](auto const& trajectories) {
        for (const auto& trajectory : trajectories) {
          moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
        }
      };

  std::future_status service_status;
  do {
    switch (service_status = service_future.wait_for(std::chrono::seconds(1)); service_status) {
      case std::future_status::deferred:
        RCLCPP_ERROR(LOGGER, "Service deferred");
        break;
      case std::future_status::timeout:
        RCLCPP_INFO(LOGGER, "Waiting for trajectory plan...");
        break;
      case std::future_status::ready:
        RCLCPP_INFO(LOGGER, "Planning service ready!");
        break;
    }
  } while (service_status != std::future_status::ready);

  auto service_response = service_future.get();
  if (service_response->response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR(LOGGER, "Planning failed with error code: %d", service_response->response.error_code.val);
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_INFO(LOGGER, "Three-segment trajectory planned successfully");
  auto trajectory = service_response->response.planned_trajectories;
  draw_trajectory_tool_path(trajectory);
  moveit_visual_tools.trigger();
  moveit_visual_tools.prompt("Press 'next' to execute the trajectory");

  using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
  auto action_client = rclcpp_action::create_client<MoveGroupSequence>(node, "/sequence_move_group");

  if (!action_client->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(LOGGER, "Action server not available");
    return -1;
  }

  moveit_msgs::msg::MotionSequenceRequest sequence_request;
  sequence_request.items.push_back(item1);
  sequence_request.items.push_back(item2);
  sequence_request.items.push_back(item3);

  auto goal_msg = MoveGroupSequence::Goal();
  goal_msg.request = sequence_request;
  goal_msg.planning_options.planning_scene_diff.is_diff = true;
  goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;

  auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
  send_goal_options.goal_response_callback = [](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle) {
    if (!goal_handle) RCLCPP_ERROR(LOGGER, "Goal rejected");
    else RCLCPP_INFO(LOGGER, "Goal accepted");
  };

  send_goal_options.result_callback = [](const GoalHandleMoveGroupSequence::WrappedResult& result) {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(LOGGER, "Execution succeeded");
        break;
      default:
        RCLCPP_ERROR(LOGGER, "Execution failed");
        break;
    }
  };

  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
  auto action_result_future = action_client->async_get_result(goal_handle_future.get());

  std::future_status action_status;
  do {
    switch (action_status = action_result_future.wait_for(std::chrono::seconds(1)); action_status) {
      case std::future_status::timeout:
        RCLCPP_INFO(LOGGER, "Executing...");
        break;
      case std::future_status::ready:
        RCLCPP_INFO(LOGGER, "Execution completed");
        break;
      default:
        RCLCPP_ERROR(LOGGER, "Action error");
        break;
    }
  } while (action_status != std::future_status::ready);

  moveit_visual_tools.prompt("Press 'next' to test cancel");
  auto cancel_goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
  sleep(5);
  auto cancel_action_result_future = action_client->async_cancel_goal(cancel_goal_handle_future.get());

  std::future_status cancel_status;
  do {
    switch (cancel_status = cancel_action_result_future.wait_for(std::chrono::seconds(1)); cancel_status) {
      case std::future_status::ready:
        RCLCPP_INFO(LOGGER, "Cancel completed");
        break;
      default:
        RCLCPP_INFO(LOGGER, "Waiting for cancel...");
        break;
    }
  } while (cancel_status != std::future_status::ready);

  rclcpp::shutdown();
  return 0;
}
