cmake_minimum_required(VERSION 3.8)
project(basic_pilz_motion_planning)

# 编译器选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pilz_industrial_motion_planner REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(moveit_msgs REQUIRED)    
find_package(shape_msgs REQUIRED)    
find_package(planning_sdk_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)

add_executable(pilz_ptp_joint src/pilz_ptp_joint.cpp)
target_include_directories(pilz_ptp_joint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pilz_ptp_joint PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pilz_ptp_joint
  rclcpp
  moveit_ros_planning_interface
  pilz_industrial_motion_planner
  geometry_msgs
  planning_sdk_msgs
)

add_executable(pilz_ptp_cartesian src/pilz_ptp_cartesian.cpp)
target_include_directories(pilz_ptp_cartesian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pilz_ptp_cartesian PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pilz_ptp_cartesian
  rclcpp
  moveit_ros_planning_interface
  pilz_industrial_motion_planner
  geometry_msgs
  tf2
  tf2_ros
  planning_sdk_msgs
)

add_executable(pilz_lin_joint src/pilz_lin_joint.cpp)
target_include_directories(pilz_lin_joint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pilz_lin_joint PUBLIC c_std_99 cxx_std_17) 

ament_target_dependencies(
  pilz_lin_joint
  "rclcpp"
  "moveit_ros_move_group"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "pilz_industrial_motion_planner"
  planning_sdk_msgs
)

add_executable(pilz_lin_cartesian src/pilz_lin_cartesian.cpp)
target_include_directories(pilz_lin_cartesian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pilz_lin_cartesian PUBLIC c_std_99 cxx_std_17) 

ament_target_dependencies(
  pilz_lin_cartesian
  "rclcpp"
  "moveit_ros_move_group"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "pilz_industrial_motion_planner"
  planning_sdk_msgs
)

add_executable(pilz_circ_cartesian src/pilz_circ_cartesian.cpp)
target_include_directories(pilz_circ_cartesian PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moveit_msgs_INCLUDE_DIRS}    
  ${shape_msgs_INCLUDE_DIRS}     
)
target_compile_features(pilz_circ_cartesian PUBLIC c_std_99 cxx_std_17)  

ament_target_dependencies(
  pilz_circ_cartesian
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pilz_industrial_motion_planner
  geometry_msgs
  moveit_msgs
  shape_msgs
  tf2_eigen
)

add_executable(pilz_sequence_demo src/pilz_sequence_demo.cpp)

target_link_libraries(pilz_sequence_demo
  moveit_core::moveit_kinematic_constraints  
)

ament_target_dependencies(pilz_sequence_demo
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  tf2
  tf2_ros
  geometry_msgs
  shape_msgs
  ament_index_cpp
  tf2_geometry_msgs
  moveit_ros_move_group        
  moveit_core                  
  moveit_visual_tools
)


install(TARGETS
  pilz_ptp_joint
  pilz_ptp_cartesian
  pilz_lin_joint
  pilz_lin_cartesian
  pilz_circ_cartesian
  pilz_sequence_demo
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()