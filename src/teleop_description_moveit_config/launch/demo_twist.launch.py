from moveit_configs_utils import MoveItConfigsBuilder
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_demo_launch(moveit_config, launch_package_path=None):
    """
    Launches a self contained demo

    launch_package_path is optional to use different launch and config packages

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
     * ros2_control_node + controller spawners
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    
    # 添加Servo控制参数
    ld.add_action(
        DeclareBooleanLaunchArg(
            "enable_servo",
            default_value=True,
            description="Enable MoveIt Servo for real-time control",
        )
    )
    
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
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

    # Fake joint driver
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )
    
    # 双臂Servo控制节点
    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="dual_arm_servo_controller",
            name="dual_arm_servo_controller",
            output="screen",
            parameters=[{
                'max_linear_velocity': 0.3,
                'max_angular_velocity': 0.5,
                'control_frame_left': 'left_wrist_pitch_link',
                'control_frame_right': 'right_wrist_pitch_link',
                'use_individual_servo_topics': True
            }],
            condition=IfCondition(LaunchConfiguration("enable_servo")),
        )
    )
    
    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="left_arm_servo",
            output="screen",
            parameters=[
                str(moveit_config.package_path / "config/left_arm_servo.yaml"),
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
            condition=IfCondition(LaunchConfiguration("enable_servo")),
        )
    )
    
    # 右臂Servo节点 - 使用正确的参数文件格式
    ld.add_action(
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="right_arm_servo",
            output="screen",
            parameters=[
                str(moveit_config.package_path / "config/right_arm_servo.yaml"),
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
            condition=IfCondition(LaunchConfiguration("enable_servo")),
        )
    )

    # 现有的控制节点
    # ld.add_action(
    #     Node(
    #         package="basic_control_topic",
    #         executable="sim_interface_switch",
    #         name="sim_interface_switch",
    #         output="screen",
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package="basic_control_topic",
    #         executable="tcp_pose_publisher",
    #         name="tcp_pose_publisher",
    #         output="screen",
    #     )
    # )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)