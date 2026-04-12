from moveit_configs_utils import MoveItConfigsBuilder
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from ament_index_python.packages import get_package_share_directory
import yaml

SERVO_MODE = "SERVO"
PILZ_MODE = "PILZ"
POSITION_MODE = "POSITION"

CURRENT_MODE = POSITION_MODE

IS_DISPLAY = True

def get_move_group_both(moveit_config):
    """配置同时支持 OMPL 和 Pilz 的 MoveGroup"""
    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    params = moveit_config.to_dict()

    # 确保配置了两个规划管道
    if "planning_pipelines" not in params:
        params["planning_pipelines"] = {
            "pipelines": ["ompl", "pilz_industrial_motion_planner"],
            "default_planning_pipeline": "ompl",  # 可以选择默认使用哪个
        }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            params,
            move_group_capabilities, 
        ],
    )
    return run_move_group_node

def get_moveit_config_both():
    """配置同时支持 OMPL 和 Pilz 的 MoveIt Config"""
    from pathlib import Path

    builder = MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config")
    builder.robot_description(file_path="config/lindenbot_teleop_description.urdf.xacro")
    builder.robot_description_semantic(file_path="config/lindenbot_teleop_description.srdf")
    builder.robot_description_kinematics(file_path="config/kinematics.yaml")
    builder.trajectory_execution(file_path="config/moveit_controllers.yaml")
    builder.planning_scene_monitor(
        publish_robot_description=True, 
        publish_robot_description_semantic=True
    )

    # 关键：加载关节限制（Pilz 必需）
    builder.joint_limits(file_path="config/joint_limits.yaml")

    # 配置规划管道：OMPL 和 Pilz
    # 注意：这里会自动查找对应的配置文件：
    # - config/ompl_planning.yaml
    # - config/pilz_industrial_motion_planner_planning.yaml
    builder.planning_pipelines(
        pipelines=["ompl", "pilz_industrial_motion_planner"],
        default_planning_pipeline="pilz_industrial_motion_planner"
    )

    moveit_config = builder.to_moveit_configs()

    # 验证 Pilz 配置文件是否存在
    pilz_config_path = Path(moveit_config.package_path) / "config" / "pilz_industrial_motion_planner_planning.yaml"
    if not pilz_config_path.exists():
        raise FileNotFoundError(
            f"❌ Pilz configuration file not found: {pilz_config_path}\n"
            f"Please create this file in your config directory!"
        )

    return moveit_config

def add_servo_launch(ld, moveit_config):
    def load_yaml(package_name, file_path):
        with open(os.path.join(get_package_share_directory(package_name), file_path), "r") as file:
            return yaml.safe_load(file)

    bool_node_list = [
        ["enable_servo", True, "Enable MoveIt Servo for real-time control"],
        ["enable_left_arm_servo", True, "Enable left arm servo control"],
        ["enable_right_arm_servo", True, "Enable right arm servo control"],
        ["enable_joint_velocity_control", True, "Enable joint velocity control"],
        ["enable_tcp_velocity_control", True, "Enable TCP velocity control"],
        ["auto_start_servo", False, "Automatically start servo nodes"],
    ]

    for name, default_val, desc in bool_node_list:
        ld.add_action(
            DeclareBooleanLaunchArg(
                name, 
                default_value=default_val,
                description=desc
            )
        )

    for arm_side in ["left","right"]:
        servo_yaml = load_yaml("teleop_description_moveit_config", f"config/{arm_side}_arm_servo.yaml")
        servo_params = {"moveit_servo": servo_yaml} if servo_yaml else {}

        acceleration_filter_update_period = {"update_period": 0.01}
        planning_group_name = {"planning_group_name": f"{arm_side}_arm_group"}

        arm_servo_node = Node(
            package="moveit_servo",
            executable="servo_node_main",
            name=f"{arm_side}_arm_servo_node",
            parameters=[
                servo_params,
                acceleration_filter_update_period,
                planning_group_name,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration(f"enable_{arm_side}_arm_servo")),
        )
        ld.add_action(arm_servo_node)

    joint_velocity_node = Node(
        package="basic_control_topic",  
        executable="joint_velocity_to_servo",
        name="joint_velocity_to_servo",
        parameters=[
            moveit_config.joint_limits,
            {"enable_left_arm": True},
            {"enable_right_arm": True},
            {"velocity_scaling_factor": 0.8},
            {"verbose_logging": True},
            {"publish_rate": 50.0},
            {"auto_start_servo": True},
            {"servo_start_retry_count": 5},
            {"servo_start_timeout": 10.0}
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_joint_velocity_control")),
    )
    ld.add_action(joint_velocity_node)

    tcp_velocity_node = Node(
        package="basic_control_topic",  
        executable="tcp_velocity_to_servo",
        name="tcp_velocity_to_servo",
        parameters=[
            {"enable_left_arm": True},
            {"enable_right_arm": True},
            {"max_linear_velocity": 0.3},         
            {"max_angular_velocity": 0.5},       
            {"verbose_logging": True}
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_tcp_velocity_control")),
    )
    ld.add_action(tcp_velocity_node)    

    servo_manager_node = Node(
        package="basic_control_topic",
        executable="servo_manager",
        name="servo_manager",
        output="screen"
    )
    ld.add_action(servo_manager_node)

def add_pilz_launch(ld, moveit_config):
    """添加 Pilz 相关节点"""
    execute_trajectory_server = Node(
        package="basic_control_topic",
        executable="execute_trajectory_server",
        name="execute_trajectory_server",
        output="screen"
    )

    tcp_trajectory_action_server = Node(
        package="basic_control_topic",
        executable="tcp_trajectory_action_server",
        name="tcp_trajectory_action_server",
        output="screen"
    )  
    ld.add_action(execute_trajectory_server)
    ld.add_action(tcp_trajectory_action_server)

def add_position_launch(ld, moveit_config):
    """添加位置控制相关节点"""
    exec_list = [
        "joint_position_delta_server",
        "joint_position_server",
        "tcp_position_action_server",
        "tcp_position_delta_action_server",
    ]

    for exe_item in exec_list:
        ld.add_action(
            Node(
                package="basic_control_topic",
                executable=exe_item,
                output="screen"
            )
        )

def add_base_launch(ld, launch_package_path, moveit_config):
    """添加基础启动配置"""
    ld.add_action(DeclareBooleanLaunchArg("db", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (launch_package_path / "launch/static_virtual_joint_tfs.launch.py")
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_package_path / "launch/rsp.launch.py")),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ), 
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            remappings=[('/joint_states', '/sim_joint_states')],
            output="screen"
        )
    )

    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="sim_interface_switch",
            name="sim_interface_switch",
            output="screen"
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

def add_common_launch(ld, moveit_config):
    """添加通用节点"""
    exec_list = [
        "tcp_pose_publisher",
    ]

    for exe_item in exec_list:
        ld.add_action(
            Node(
                package="basic_control_topic",
                executable=exe_item,
                output="screen"
            )
        )

def generate_launch_description():
    """生成支持 OMPL + Pilz 双规划器的启动描述"""

    # 使用同时支持 OMPL 和 Pilz 的配置
    moveit_config = get_moveit_config_both()
    launch_package_path = moveit_config.package_path

    # 打印配置信息用于调试
    print("\n" + "="*80)
    print("MoveIt Configuration Summary")
    print("="*80)
    print(f"Package path: {launch_package_path}")
    print(f"Planning pipelines: {moveit_config.planning_pipelines.get('pipelines', [])}")
    print(f"Default pipeline: {moveit_config.planning_pipelines.get('default_planning_pipeline', 'N/A')}")

    # 检查配置文件
    from pathlib import Path
    config_files = {
        "OMPL": Path(launch_package_path) / "config" / "ompl_planning.yaml",
        "Pilz": Path(launch_package_path) / "config" / "pilz_industrial_motion_planner_planning.yaml",
        "Joint Limits": Path(launch_package_path) / "config" / "joint_limits.yaml",
    }

    print("\nConfiguration files:")
    for name, path in config_files.items():
        status = "✓" if path.exists() else "✗ MISSING"
        print(f"  {status} {name}: {path.name}")
    print("="*80 + "\n")

    ld = LaunchDescription()

    # 使用同时支持 OMPL 和 Pilz 的 MoveGroup
    move_group_node = get_move_group_both(moveit_config)
    ld.add_action(move_group_node)

    # 添加所有功能模块
    add_base_launch(ld, launch_package_path, moveit_config)
    add_servo_launch(ld, moveit_config)
    add_position_launch(ld, moveit_config)
    add_pilz_launch(ld, moveit_config)
    add_common_launch(ld, moveit_config)

    return ld