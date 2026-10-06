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

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config")
        .robot_description(file_path="config/lindenbot_teleop_description.urdf.xacro")
        .robot_description_semantic(file_path="config/lindenbot_teleop_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True )
        .planning_pipelines(pipelines=["ompl","pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Starts Pilz Industrial Motion Planner MoveGroupSequenceAction and MoveGroupSequenceService servers
    # Start the actual move_group node/action server
    # moveit_config = MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)

def load_yaml(package_name, file_path):
    """加载YAML配置文件"""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def add_servo_launch(ld, moveit_config):


    # move_group_capabilities = {
    #     "capabilities": "ompl/MoveGroupSequenceAction ompl/MoveGroupSequenceService"
    # }
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            #move_group_capabilities,
        ],
        #env={'DISPLAY': ''}
    )

    ld.add_action( run_move_group_node)

    ld.add_action(
        DeclareBooleanLaunchArg(
            "enable_servo", 
            default_value=True,
            description="Enable MoveIt Servo for real-time control"
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "enable_left_arm_servo",
            default_value=True, 
            description="Enable left arm servo control"
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "enable_right_arm_servo",
            default_value=True,
            description="Enable right arm servo control"
        )
    )
    
    ld.add_action(
        DeclareBooleanLaunchArg(
            "enable_joint_velocity_control",
            default_value=True,
            description="Enable joint velocity control"
        )
    )
    
    ld.add_action(
        DeclareBooleanLaunchArg(
            "enable_tcp_velocity_control",
            default_value=True,
            description="Enable TCP velocity control"
        )
    )
    
    ld.add_action(
        DeclareBooleanLaunchArg(
            "auto_start_servo",
            default_value=True,
            description="Automatically start servo nodes"
        )
    )

    left_servo_yaml = load_yaml("teleop_description_moveit_config", "config/left_arm_servo.yaml")
    right_servo_yaml = load_yaml("teleop_description_moveit_config", "config/right_arm_servo.yaml")

    left_servo_params = {"moveit_servo": left_servo_yaml} if left_servo_yaml else {}
    right_servo_params = {"moveit_servo": right_servo_yaml} if right_servo_yaml else {}
    
    acceleration_filter_update_period = {"update_period": 0.01}
    left_planning_group_name = {"planning_group_name": "left_arm_group"}
    right_planning_group_name = {"planning_group_name": "right_arm_group"}

    right_arm_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_arm_servo_node",
        parameters=[
            right_servo_params,
            acceleration_filter_update_period,
            right_planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_right_arm_servo")),
    )
    ld.add_action(right_arm_servo_node)

    left_arm_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_arm_servo_node",
        parameters=[
            left_servo_params,
            acceleration_filter_update_period,
            left_planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_left_arm_servo")),
    )
    ld.add_action(left_arm_servo_node)

    # 关节速度控制节点
    joint_velocity_node = Node(
        package="basic_control_topic",  
        executable="joint_velocity_to_servo",
        name="joint_velocity_to_servo",
        parameters=[
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

    # TCP速度控制节点
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

    # 添加Servo启动服务调用（在Servo节点启动后延迟执行）
    def start_servo_after_node_start():
        start_left_servo = ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/left_arm_servo_node/start_servo',
                'std_srvs/srv/Trigger',
                '{}'
            ],
            name='start_left_arm_servo',
            output='screen',
            condition=IfCondition(LaunchConfiguration("auto_start_servo"))
        )
        
        start_right_servo = ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call',
                '/right_arm_servo_node/start_servo',
                'std_srvs/srv/Trigger',
                '{}'
            ],
            name='start_right_arm_servo',
            output='screen',
            condition=IfCondition(LaunchConfiguration("auto_start_servo"))
        )
        
        return [start_left_servo, start_right_servo]

    ld.add_action(
        RegisterEventHandler(
            OnProcessStart(
                target_action=left_arm_servo_node,
                on_start=[
                    TimerAction(
                        period=3.0,
                        actions=start_servo_after_node_start()
                    )
                ]
            )
        )
    )

def add_pilz_launch(ld, moveit_config):

    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
        #env={'DISPLAY': ''}
    )

    ld.add_action( run_move_group_node)

    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="execute_trajectory_server",
        )
    )
    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="tcp_trajectory_action_server",
        )
    )

def add_joint_tcp_launch(ld, moveit_config):

    move_group_capabilities = {
        "capabilities": ""
    }
    # 添加
    ompl_planning_config = {
          "default_planning_pipeline": "ompl",
          "ompl.planning_plugins": "ompl_interface/OMPLPlanner",
          "ompl.request_adapters": "",
          "ompl.response_adapters": "default_planning_response_adapters/AddTimeOptimalParameterization default_planning_response_adapters/ValidateSolution",
      }
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            ompl_planning_config,  # 添加这行
        ],
        
        # remappings=[
        #     ("/joint_states", "/cerebellum_sdk/arm/joint_states"),
        # ],
    )

    ld.add_action( run_move_group_node)
    ld.add_action(

        Node(
            package="basic_control_topic",
            executable="joint_position_delta_server",
        )
    )

    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="joint_position_server",
        )
    )


    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="tcp_pose_publisher",
        )
    )


    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="tcp_position_action_server",
        )
    )


    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="tcp_position_delta_action_server",
        )
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

    # add_pilz_launch(ld, moveit_config)
    # add_servo_launch(ld, moveit_config)
    add_joint_tcp_launch(ld, moveit_config)

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
    

    return ld
