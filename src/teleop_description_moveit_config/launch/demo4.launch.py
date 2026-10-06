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
    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }
    
    params = moveit_config.to_dict()
    
    if "planning_pipelines" not in params:
        params["planning_pipelines"] = {
            "pipelines": ["ompl", "pilz_industrial_motion_planner"],
            "default_planning_pipeline": "pilz_industrial_motion_planner",  
        }
    
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            params,
            move_group_capabilities, 
        ],
        # env={'DISPLAY': ''} if not IS_DISPLAY else {}
    )
    return run_move_group_node

# def get_move_group_ompl(moveit_config):
#     run_move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#         ],
#         env={'DISPLAY': ''}
#     )

#     if IS_DISPLAY:
#         run_move_group_node = Node(
#             package="moveit_ros_move_group",
#             executable="move_group",
#             output="screen",
#             parameters=[
#                 moveit_config.to_dict(),
#             ]
#         )
#     return run_move_group_node

# def get_move_group_pilz(moveit_config):
#     move_group_capabilities = {
#         "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
#     }
#     run_move_group_node = Node(
#         package="moveit_ros_move_group",
#         executable="move_group",
#         output="screen",
#         parameters=[
#             moveit_config.to_dict(),
#             move_group_capabilities,
#         ],
#         env={'DISPLAY': ''}
#     )

#     if IS_DISPLAY:
#         run_move_group_node = Node(
#             package="moveit_ros_move_group",
#             executable="move_group",
#             output="screen",
#             parameters=[
#                 moveit_config.to_dict(),
#                 move_group_capabilities,
#             ]
#         )
#     return run_move_group_node

# def get_moveit_config_ompl():
#     return (
#         MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config")
#         .robot_description(file_path="config/lindenbot_teleop_description.urdf.xacro")
#         .robot_description_semantic(file_path="config/lindenbot_teleop_description.srdf")
#         .robot_description_kinematics(file_path="config/kinematics.yaml")
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .planning_scene_monitor(
#             publish_robot_description=True, publish_robot_description_semantic=True )
#         .planning_pipelines(pipelines=["ompl"])
#         .to_moveit_configs()
#     )

# def get_moveit_config_pilz():
#     return (
#         MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config")
#         .robot_description(file_path="config/lindenbot_teleop_description.urdf.xacro")
#         .robot_description_semantic(file_path="config/lindenbot_teleop_description.srdf")
#         .robot_description_kinematics(file_path="config/kinematics.yaml")
#         .trajectory_execution(file_path="config/moveit_controllers.yaml")
#         .planning_scene_monitor(
#             publish_robot_description=True, publish_robot_description_semantic=True )
#         .planning_pipelines(pipelines=["pilz_industrial_motion_planner"])
#         .to_moveit_configs()
#     )

def get_moveit_config_both():
    return (
        MoveItConfigsBuilder("lindenbot_teleop_description", package_name="teleop_description_moveit_config")
        .robot_description(file_path="config/lindenbot_teleop_description.urdf.xacro")
        .robot_description_semantic(file_path="config/lindenbot_teleop_description.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

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

        # start_servo = ExecuteProcess(
        #     cmd=[
        #         'ros2', 'service', 'call', f'/{arm_side}_arm_servo_node/start_servo',
        #         'std_srvs/srv/Trigger', '{}'
        #     ],
        #     name=f'start_{arm_side}_arm_servo',
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration("auto_start_servo"))
        # )

        # ld.add_action(
        #     RegisterEventHandler(
        #         OnProcessStart(
        #             target_action=arm_servo_node,
        #             on_start=[ TimerAction( period=3.0, actions=start_servo )  ]
        #         )
        #     )
        # )

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
    #pilz
    execute_trajectory_server = Node(
        package="basic_control_topic",
        executable="execute_trajectory_server",
        name="execute_trajectory_server",
        parameters=[
            {"use_pilz_planner": True},
            {"planning_pipeline_id": "pilz_industrial_motion_planner"},
            #{"pilz_planner_type": "LIN"},
        ],
        output="screen"
    )
    
    tcp_trajectory_action_server = Node(
        package="basic_control_topic",
        executable="tcp_trajectory_action_server",
        name="tcp_trajectory_action_server",
        parameters=[
            {"use_pilz_planner": True},
            {"planning_pipeline_id": "pilz_industrial_motion_planner"},
           #{"pilz_planner_type": "PTP"}, 
        ],
        output="screen"
    )  
    ld.add_action(execute_trajectory_server)
    ld.add_action(tcp_trajectory_action_server)

def add_position_launch(ld, moveit_config):
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
                #ompl
                parameters=[
                    {"use_ompl_planner": True},
                    {"planning_pipeline_id": "ompl"},
                ]
            )
        )

def add_base_launch(ld, launch_package_path, moveit_config):
    ld.add_action(DeclareBooleanLaunchArg( "db", default_value=False,))
    ld.add_action( DeclareBooleanLaunchArg("debug", default_value=False ))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))

    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = ( launch_package_path / "launch/static_virtual_joint_tfs.launch.py" )
    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription( PythonLaunchDescriptionSource(str(virtual_joints_launch)), )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource( str(launch_package_path / "launch/rsp.launch.py") ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription( 
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ), condition=IfCondition(LaunchConfiguration("use_rviz")),
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
    exec_list = [
        "tcp_pose_publisher",
        #"sim_interface_switch"
    ]
    
    for exe_item in exec_list:
        ld.add_action(
            Node(
                package="basic_control_topic",
                executable=exe_item,
            )
        )

ADD_LAUNCH_FUNC = {
    SERVO_MODE : add_servo_launch,
    PILZ_MODE : add_pilz_launch,
    POSITION_MODE : add_position_launch
}

def generate_launch_description():
    assert CURRENT_MODE in [SERVO_MODE, PILZ_MODE, POSITION_MODE], \
        f"CURRENT_MODE must in [{SERVO_MODE}, {PILZ_MODE}, {POSITION_MODE}], but is {CURRENT_MODE} "
    
    # moveit_config = None

    # if CURRENT_MODE in [SERVO_MODE, POSITION_MODE]:
    #     moveit_config = get_moveit_config_ompl()
    
    # if CURRENT_MODE in [PILZ_MODE]:
    #     moveit_config = get_moveit_config_pilz()
    
    moveit_config = get_moveit_config_both()

    launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    # move_group_node = None

    # if CURRENT_MODE in [SERVO_MODE, POSITION_MODE]:
    #     move_group_node = get_move_group_ompl(moveit_config)
    
    # if CURRENT_MODE in [PILZ_MODE]:
    #     move_group_node = get_move_group_pilz(moveit_config)
    move_group_node = get_move_group_both(moveit_config)

    ld.add_action( move_group_node)

    add_base_launch(ld, launch_package_path, moveit_config)
    # add_servo_launch(ld, moveit_config)
    # add_position_launch(ld, moveit_config)
    add_pilz_launch(ld, moveit_config)
    add_common_launch(ld, moveit_config)

    return ld