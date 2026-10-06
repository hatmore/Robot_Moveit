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

# 使用统一入口点（单进程）替代多个独立节点
USE_UNIFIED_NODE = True


def _sync_initial_positions(yaml_path, topic='/cerebellum_sdk/arm/joint_states', timeout=10):
    """
    从实机关节状态话题读取当前位置，写入 initial_positions.yaml。
    在 launch 加载阶段同步执行，确保 ros2_control_node 启动前完成。
    使用 rclpy 直接订阅，不依赖 ros2 daemon。
    """
    import time

    print(f'[sync] Reading joint states from {topic} (timeout={timeout}s)...')

    try:
        import rclpy
        from rclpy.node import Node as RclpyNode
        from sensor_msgs.msg import JointState
    except ImportError as e:
        print(f'[sync] WARN: Cannot import rclpy modules: {e}')
        return

    # 尝试初始化 rclpy（launch 系统可能已经初始化过）
    own_init = False
    try:
        rclpy.init()
        own_init = True
    except RuntimeError:
        pass

    try:
        node = RclpyNode('_sync_initial_positions_tmp')
        received_msg = None

        def cb(msg):
            nonlocal received_msg
            received_msg = msg

        node.create_subscription(JointState, topic, cb, 10)

        start_time = time.time()
        while received_msg is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(node, timeout_sec=0.5)

        node.destroy_node()
    except Exception as e:
        print(f'[sync] WARN: Error during subscription: {e}')
        return
    finally:
        if own_init:
            try:
                rclpy.shutdown()
            except Exception:
                pass

    if received_msg is None:
        print(f'[sync] WARN: Timeout reading {topic}, keeping existing initial_positions.yaml')
        return

    names = list(received_msg.name)
    positions = list(received_msg.position)
    if not names or len(names) != len(positions):
        print(f'[sync] WARN: Invalid joint state data (names={len(names)}, positions={len(positions)})')
        return

    # URDF 硬限位，用于检查实机关节是否超限
    urdf_limits = {
        'left_shoulder_pitch_joint':  (-4.36332,   1.22173),
        'left_shoulder_roll_joint':   (-0.733038,  2.879793),
        'left_shoulder_yaw_joint':    (-2.7925268, 2.7925268),
        'left_elbow_joint':           (-2.338741,  2.338741),
        'left_wrist_roll_joint':      (-2.8448867, 2.8448867),
        'left_wrist_yaw_joint':       (-0.959931,  0.959931),
        'left_wrist_pitch_joint':     (-1.570796,  1.570796),
        'right_shoulder_pitch_joint': (-1.22173,   4.36332),
        'right_shoulder_roll_joint':  (-2.879793,  0.733038),
        'right_shoulder_yaw_joint':   (-2.7925268, 2.7925268),
        'right_elbow_joint':          (-2.338741,  2.338741),
        'right_wrist_roll_joint':     (-2.8448867, 2.8448867),
        'right_wrist_yaw_joint':      (-0.959931,  0.959931),
        'right_wrist_pitch_joint':    (-1.570796,  1.570796),
    }

    import sys

    initial = {}
    out_of_limits = []
    for name, pos in zip(names, positions):
        raw = float(pos)
        if name in urdf_limits:
            lo, hi = urdf_limits[name]
            if raw < lo or raw > hi:
                out_of_limits.append((name, raw, lo, hi))
        initial[name] = raw

    if out_of_limits:
        print('[sync] ERROR: Real robot joint states out of URDF limits! Aborting launch.')
        for name, pos, lo, hi in out_of_limits:
            print(f'[sync]   {name} = {pos:.4f}, limits = [{lo:.4f}, {hi:.4f}]')
        sys.exit(1)

    # 补充 vacuum 关节（SDK 不发布）
    # for prefix in ['left', 'right']:
    #     key = f'{prefix}_vacuum_control_joint'
    #     if key not in initial:
    #         initial[key] = 0.0

    data = {'initial_positions': initial}
    with open(yaml_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

    print(f'[sync] Saved {len(initial)} joint positions to {yaml_path}')
    for name in sorted(initial):
        print(f'[sync]   {name}: {initial[name]:.6f}')


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

    # servo_node_main 必须始终启动，VelocityControllersNode 依赖它
    for arm_side in ["left", "right"]:
        servo_yaml = load_yaml("teleop_description_moveit_config", f"config/{arm_side}_arm_servo.yaml")
        servo_params = {"moveit_servo": servo_yaml} if servo_yaml else {}

        arm_servo_node = Node(
            package="moveit_servo",
            executable="servo_node_main",
            name=f"{arm_side}_arm_servo_node",
            parameters=[
                servo_params,
                {"update_period": 0.01},
                {"planning_group_name": f"{arm_side}_arm_group"},
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
            output="screen",
        )
        ld.add_action(arm_servo_node)

    if USE_UNIFIED_NODE:
        # 应用层（joint_velocity_to_servo / tcp_velocity_to_servo / servo_manager）
        # 已合并到 basic_control_node，只需启动 servo_node_main
        return

    # 非统一模式：启动应用层独立节点
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
            {"servo_start_timeout": 10.0},
        ],
        output="screen",
    )
    ld.add_action(joint_velocity_node)

    tcp_velocity_node = Node(
        package="basic_control_topic",
        executable="tcp_velocity_to_servo",
        name="tcp_velocity_to_servo",
        parameters=[
            {"enable_left_arm": True},
            {"enable_right_arm": True},
            {"max_linear_velocity": 2.0},
            {"max_angular_velocity": 2.5},
            {"verbose_logging": True},
        ],
        output="screen",
    )
    ld.add_action(tcp_velocity_node)

    servo_manager_node = Node(
        package="basic_control_topic",
        executable="servo_manager",
        name="servo_manager",
        output="screen",
    )
    ld.add_action(servo_manager_node)

def add_pilz_launch(ld, moveit_config):
    """添加 Pilz 相关节点"""
    if USE_UNIFIED_NODE:
        # 统一入口点已包含轨迹控制功能
        # 注意：当前 basic_control_node 未包含 tcp_trajectory_action_server 和 execute_trajectory_server
        # 这些需要独立启动或后续添加到统一入口点
        tcp_trajectory_action_server = Node(
            package="basic_control_topic",
            executable="tcp_trajectory_action_server",
            name="tcp_trajectory_action_server",
            output="screen"
        )
        execute_trajectory_server = Node(
            package="basic_control_topic",
            executable="execute_trajectory_server",
            name="execute_trajectory_server",
            output="screen"
        )
        ld.add_action(tcp_trajectory_action_server)
        ld.add_action(execute_trajectory_server)
        return

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
    if USE_UNIFIED_NODE:
        # 统一入口点已包含位置控制功能
        return

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

    # joint_limits_server 需要 robot_description 才能解析 URDF 硬限位
    ld.add_action(
        Node(
            package="basic_control_topic",
            executable="joint_limits_server",
            output="screen",
            parameters=[moveit_config.robot_description],
        )
    )

def add_base_launch(ld, launch_package_path, moveit_config):
    """添加基础启动配置"""
    ld.add_action(DeclareBooleanLaunchArg("db", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=False))

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

    if not USE_UNIFIED_NODE:
        # 统一入口点已包含 sim_interface_switch
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
    if USE_UNIFIED_NODE:
        # 使用统一入口点 - 所有功能合并到一个进程
        basic_control_node = Node(
            package="basic_control_topic",
            executable="basic_control_node",
            name="basic_control_node",
            parameters=[
                moveit_config.robot_description,
                moveit_config.joint_limits,
                {"enable_left_arm": True},
                {"enable_right_arm": True},
                {"max_linear_velocity": 2.0},
                {"max_angular_velocity": 2.5},
                {"velocity_scaling_factor": 0.8},
                {"verbose_logging": True},
            ],
            output="screen"
        )
        ld.add_action(basic_control_node)
        return

    # 原有方式：多个独立节点
    exec_list = [
        "tcp_pose_publisher",
        "heart_beat",
    ]

    for exe_item in exec_list:
        ld.add_action(
            Node(
                package="basic_control_topic",
                executable=exe_item,
                output="screen"
            )
        )


def add_initial_pose_launch(ld):
    """系统完全启动 3 秒后，将双臂移动到初始姿态"""
    left_goal = (
        "{timestamp: {sec: 0, nanosec: 0},"
        " target_state: {"
        "header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},"
        " name: ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint',"
        " 'left_shoulder_yaw_joint', 'left_elbow_joint',"
        " 'left_wrist_roll_joint', 'left_wrist_yaw_joint', 'left_wrist_pitch_joint'],"
        " position: [0.7854, -0.6109, -0.3491, -1.5708, 0.2617, 0.0, -0.8727],"
        " velocity: [], effort: []}}"
    )
    right_goal = (
        "{timestamp: {sec: 0, nanosec: 0},"
        " target_state: {"
        "header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},"
        " name: ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint',"
        " 'right_shoulder_yaw_joint', 'right_elbow_joint',"
        " 'right_wrist_roll_joint', 'right_wrist_yaw_joint', 'right_wrist_pitch_joint'],"
        " position: [-0.7854, 0.6109, 0.3491, 1.5708, -0.2617, 0.0, 0.8726],"
        " velocity: [], effort: []}}"
    )

    pass

def generate_launch_description():
    """生成支持 OMPL + Pilz 双规划器的启动描述"""

    # 必须在 get_moveit_config_both() 之前同步：
    # MoveItConfigsBuilder 展开 xacro 时会读取 initial_positions.yaml，
    # 展开后 robot_description 已固化 initial_value，之后再改 yaml 无效。
    from ament_index_python.packages import get_package_share_directory
    _yaml_path = os.path.join(
        get_package_share_directory("teleop_description_moveit_config"),
        "config", "initial_positions.yaml"
    )
    _sync_initial_positions(_yaml_path)

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
    add_initial_pose_launch(ld)

    return ld