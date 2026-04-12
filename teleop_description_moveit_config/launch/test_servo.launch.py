# fixed_demo_launch.py
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

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_demo_launch(moveit_config, launch_package_path=None):
    """
    Fixed launch file with simplified Servo configuration
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    
    # 原有参数
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
    
    # Servo相关参数
    ld.add_action(
        DeclareBooleanLaunchArg(
            "use_servo",
            default_value=False,  # 默认禁用，等测试成功后再启用
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
            remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
            ],
            output="screen",
        )
    )
    

    # 新加节点 - 原有的控制节点
    # ld.add_action(
    #     Node(
    #         package="basic_control_topic",
    #         executable="joint_position_delta_server",
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package="basic_control_topic",
    #         executable="joint_position_server",
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package="basic_control_topic",
    #         executable="tcp_pose_publisher",
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package="basic_control_topic",
    #         executable="tcp_position_action_server",
    #     )
    # )

    # ld.add_action(
    #     Node(
    #         package="basic_control_topic",
    #         executable="tcp_position_delta_action_server",
    #     )
    # )

    # TCP速度控制器节点 - 先不启动，等Servo稳定后再添加
    '''
    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="tcp_velocity_controller",
            name="tcp_velocity_controller",
            parameters=[{
                "max_linear_speed": 0.2,
                "max_angular_speed": 0.8,
            }],
            condition=IfCondition(LaunchConfiguration("use_servo")),
            output="screen",
        )
    )
    '''

    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="sim_interface_switch",
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/spawn_controllers.launch.py")
            ),
        )
    )

    # 简化的Servo节点配置 - 不使用YAML文件
    left_arm_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_arm_servo_node",
        parameters=[
            # 最小化的Servo配置
            {"moveit_servo.publish_period": 0.01},
            {"moveit_servo.command_in_type": "speed_units"},
            {"moveit_servo.scale.linear": 0.1},  # 降低速度限制
            {"moveit_servo.scale.rotational": 0.5},
            {"moveit_servo.move_group_name": "left_arm_group"},
            {"moveit_servo.command_out_topic": "/left_arm_group_controller/joint_trajectory"},
            {"moveit_servo.is_primary_planning_scene_monitor": False},  # 设为False避免冲突
            
            # MoveIt配置
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        remappings=[
            ("/servo_node/delta_twist_cmds", "/left_arm/servo_node/delta_twist_cmds"),
        ],
        condition=IfCondition(LaunchConfiguration("use_servo")),
        output="screen",
    )

    # 右臂Servo节点 - 暂时不启动，先测试左臂
    '''
    right_arm_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_arm_servo_node",
        parameters=[
            {"moveit_servo.publish_period": 0.01},
            {"moveit_servo.command_in_type": "speed_units"},
            {"moveit_servo.scale.linear": 0.1},
            {"moveit_servo.scale.rotational": 0.5},
            {"moveit_servo.move_group_name": "right_arm_group"},
            {"moveit_servo.command_out_topic": "/right_arm_group_controller/joint_trajectory"},
            {"moveit_servo.is_primary_planning_scene_monitor": False},
            
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        remappings=[
            ("/servo_node/delta_twist_cmds", "/right_arm/servo_node/delta_twist_cmds"),
        ],
        condition=IfCondition(LaunchConfiguration("use_servo")),
        output="screen",
    )
    '''

    ld.add_action(left_arm_servo_node)
    # ld.add_action(right_arm_servo_node)  # 暂时注释掉右臂

    return ld


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)