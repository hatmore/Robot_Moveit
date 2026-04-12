cmake_minimum_required(VERSION 3.8)
project(basic_control_topic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)  # 新增C++17标准（Eigen/KDL需要）
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)

find_package(planning_sdk_msgs REQUIRED)
find_package(cerebellum_sdk_msg REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(moveit_servo REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(control_msgs REQUIRED)

find_package(moveit_kinematics REQUIRED)  
find_package(kdl_parser REQUIRED)        
find_package(urdf REQUIRED)              
find_package(orocos_kdl REQUIRED)        
find_package(Eigen3 REQUIRED)
find_package(controller_manager_msgs REQUIRED)
       
include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# include_directories(${PROJECT_SOURCE_DIR}/../moveit_servo/include)

add_executable(tcp_pose_publisher src/tcp_pose_publisher.cpp)
ament_target_dependencies(tcp_pose_publisher
  rclcpp tf2_ros tf2 geometry_msgs builtin_interfaces planning_sdk_msgs cerebellum_sdk_msg
)
install(TARGETS tcp_pose_publisher DESTINATION lib/${PROJECT_NAME})



add_executable(joint_position_server src/joint_position_server.cpp)
ament_target_dependencies(joint_position_server
  rclcpp
  rclcpp_action
  sensor_msgs
  builtin_interfaces
  planning_sdk_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  cerebellum_sdk_msg
  rosidl_typesupport_cpp
  controller_manager_msgs
  std_msgs
)
target_link_libraries(joint_position_server
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)
install(TARGETS joint_position_server DESTINATION lib/${PROJECT_NAME})

add_executable(joint_position_delta_server src/joint_position_delta_server.cpp)
ament_target_dependencies(joint_position_delta_server
  rclcpp
  rclcpp_action
  planning_sdk_msgs
  sensor_msgs
  builtin_interfaces
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  cerebellum_sdk_msg
  rosidl_typesupport_cpp
)
target_link_libraries(joint_position_delta_server
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)
install(TARGETS joint_position_delta_server DESTINATION lib/${PROJECT_NAME})

add_executable(joint_velocity_to_servo src/joint_velocity_to_servo.cpp)
ament_target_dependencies(joint_velocity_to_servo
  rclcpp
  rclcpp_action
  planning_sdk_msgs
  trajectory_msgs
  sensor_msgs
  control_msgs
  moveit_servo
  moveit_core
  moveit_ros_planning
  moveit_kinematics
  std_srvs
)
target_link_libraries(joint_velocity_to_servo
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)
install(TARGETS joint_velocity_to_servo DESTINATION lib/${PROJECT_NAME})

add_executable(tcp_position_action_server src/tcp_position_action_server.cpp)
ament_target_dependencies(tcp_position_action_server
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  planning_sdk_msgs
  rosidl_typesupport_cpp
  cerebellum_sdk_msg
)
target_link_libraries(tcp_position_action_server
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)
install(TARGETS tcp_position_action_server DESTINATION lib/${PROJECT_NAME})

add_executable(tcp_position_delta_action_server src/tcp_position_delta_action_server.cpp)
ament_target_dependencies(tcp_position_delta_action_server
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_ros
  planning_sdk_msgs
  rosidl_typesupport_cpp
  cerebellum_sdk_msg
)
target_link_libraries(tcp_position_delta_action_server
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)
install(TARGETS tcp_position_delta_action_server DESTINATION lib/${PROJECT_NAME})


add_executable(tcp_velocity_to_servo src/tcp_velocity_to_servo.cpp)
ament_target_dependencies(
  tcp_velocity_to_servo
  rclcpp
  tf2_ros
  planning_sdk_msgs
  trajectory_msgs
  sensor_msgs
  moveit_servo
  moveit_core
  moveit_ros_planning
  moveit_kinematics
)   
# target_include_directories(tcp_velocity_controller PRIVATE ${moveit_servo_INCLUDE_DIRS})
target_link_libraries(tcp_velocity_to_servo
  rclcpp::rclcpp
  moveit_msgs::moveit_msgs__rosidl_typesupport_cpp
)
install(TARGETS tcp_velocity_to_servo DESTINATION lib/${PROJECT_NAME})

add_executable(servo_manager src/servo_manager.cpp)
ament_target_dependencies(servo_manager
  rclcpp
  planning_sdk_msgs
  std_srvs
)
install(TARGETS servo_manager DESTINATION lib/${PROJECT_NAME})

add_executable(heart_beat src/heart_beat.cpp)
ament_target_dependencies(heart_beat
  rclcpp
  planning_sdk_msgs
  std_msgs
)
install(TARGETS heart_beat DESTINATION lib/${PROJECT_NAME})

add_executable(tcp_trajectory_action_server src/tcp_trajectory_action_server.cpp)
ament_target_dependencies(tcp_trajectory_action_server
  rclcpp
  rclcpp_action
  moveit_ros_move_group
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  planning_sdk_msgs
  geometry_msgs
  std_msgs
  shape_msgs
  moveit_msgs
  tf2_geometry_msgs
  cerebellum_sdk_msg
)

install(TARGETS
  tcp_trajectory_action_server
  DESTINATION lib/${PROJECT_NAME}
)
add_executable(sim_interface_switch src/sim_interface_switch.cpp)
ament_target_dependencies(sim_interface_switch
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_ros
  planning_sdk_msgs
  rosidl_typesupport_cpp
  controller_manager_msgs
  std_srvs
  sensor_msgs
  cerebellum_sdk_msg
)
target_link_libraries(sim_interface_switch
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)
install(TARGETS sim_interface_switch DESTINATION lib/${PROJECT_NAME})

add_executable(execute_trajectory_server
  src/execute_trajectory_server.cpp
)

ament_target_dependencies(execute_trajectory_server
  rclcpp
  rclcpp_action
  planning_sdk_msgs
  builtin_interfaces
  std_msgs
  trajectory_msgs
  control_msgs
  cerebellum_sdk_msg
  controller_manager_msgs
  sensor_msgs
)

install(TARGETS
  execute_trajectory_server
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(frame_registration_server 
  src/frame_registration_server.cpp)
ament_target_dependencies(frame_registration_server
  rclcpp planning_sdk_msgs tf2_ros geometry_msgs)

add_executable(frame_registration_client
  src/frame_registration_client.cpp)
ament_target_dependencies(frame_registration_client
  rclcpp planning_sdk_msgs)

install(TARGETS
  frame_registration_server
  frame_registration_client
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(joint_limits_server src/joint_limits_server.cpp)
ament_target_dependencies(joint_limits_server
  rclcpp
  planning_sdk_msgs
  urdf
)
install(TARGETS joint_limits_server DESTINATION lib/${PROJECT_NAME})

# add_executable(pilz_planner_manager src/pilz_planner_manager.cpp)
# ament_target_dependencies(pilz_planner_manager
#   rclcpp
#   rclcpp_action
#   planning_sdk_msgs
#   std_srvs
#   moveit_msgs
# )
# install(TARGETS pilz_planner_manager DESTINATION lib/${PROJECT_NAME})

# ============================================================================
# 新增：统一入口点 - 单进程多节点
# ============================================================================
# 组件源文件
set(COMPONENT_SOURCES
  src/components/position_servers_node.cpp
  src/components/velocity_controllers_node.cpp
)

# 统一入口点
add_executable(basic_control_node
  src/basic_control_node.cpp
  ${COMPONENT_SOURCES}
)

ament_target_dependencies(basic_control_node
  rclcpp
  rclcpp_action
  planning_sdk_msgs
  cerebellum_sdk_msg
  sensor_msgs
  geometry_msgs
  builtin_interfaces
  std_msgs
  trajectory_msgs
  control_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  moveit_servo
  moveit_kinematics
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  std_srvs
  controller_manager_msgs
)

target_link_libraries(basic_control_node
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
)

install(TARGETS basic_control_node DESTINATION lib/${PROJECT_NAME})

ament_package()
