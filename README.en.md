[中文](README.md) | **English**

# Robot_Moveit

A **ROS 2 Humble + MoveIt 2** workspace for a dual-arm robot. On top of MoveIt it exposes a stable ROS interface (actions / services / topics) so that a higher-level task system can request preset poses, joint and end-effector motions, trajectory execution and end-effector velocity servoing. It also provides module heartbeats, independent left/right-arm scheduling and a switch between simulated and real hardware interfaces. Planning is supported through **OMPL**, the **Pilz industrial motion planner** and **MoveIt Servo**.

## Layout

```
Robot_Moveit/                          # colcon workspace root
├── src/
│   ├── basic_control_topic/           # control layer: public action / service / topic nodes
│   ├── basic_pilz_motion_planning/    # Pilz PTP / LIN / CIRC / sequence examples
│   ├── kernel_msgs/                   # interface definitions
│   │   ├── planning_sdk_msgs/         #   planning-layer interfaces (action / srv / msg)
│   │   ├── cerebellum_sdk_msg/        #   interfaces to the low-level Cerebellum SDK
│   │   └── power_borad_communication/ #   power board status
│   ├── mpc_position_controller/       # position-level MPC controller plugin for ros2_control
│   └── teleop_description_moveit_config/  # MoveIt config (URDF/SRDF, controllers, launch)
├── scripts/                           # build, packaging, release and on-robot start scripts
├── .gitignore
└── README.md
```

### Packages

| Package | Description |
|---------|-------------|
| `basic_control_topic` | `basic_control_node` composite node plus standalone nodes: `joint_position_server`, `joint_position_delta_server`, `tcp_position_action_server`, `tcp_position_delta_action_server`, `tcp_trajectory_action_server`, `execute_trajectory_server`, `joint_velocity_to_servo`, `tcp_velocity_to_servo`, `servo_manager`, `tcp_pose_publisher`, `joint_limits_server`, `frame_registration_server/client`, `heart_beat`, `sim_interface_switch`, `initial_joint_positions`. See the [package README](src/basic_control_topic/README.md) |
| `basic_pilz_motion_planning` | Example programs `pilz_ptp_joint`, `pilz_ptp_cartesian`, `pilz_lin_joint`, `pilz_lin_cartesian`, `pilz_circ_cartesian`, `pilz_sequence_demo` |
| `kernel_msgs/*` | Interface definitions only, see [kernel_msgs/README.md](src/kernel_msgs/README.md) |
| `mpc_position_controller` | Drop-in replacement for `JointTrajectoryController`; theory and parameters in the [package README](src/mpc_position_controller/README.md) |
| `teleop_description_moveit_config` | `demo.launch.py` (basic), `demo2.launch.py` (position control), `demo3.launch.py` (full: OMPL + Pilz + Servo), `demo4.launch.py` (Pilz trajectories); comparison in the [package README](src/teleop_description_moveit_config/README.md) |

## Requirements

- Ubuntu 22.04, ROS 2 **Humble**
- MoveIt 2 (`ros-humble-moveit`, `ros-humble-moveit-servo`)
- ros2_control / ros2_controllers
- everything else via `rosdep`:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
git clone https://github.com/hatmore/Robot_Moveit.git
cd Robot_Moveit
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Or use the unified build script (native, Docker x86_64 or Docker ARM64; artefacts include branch, commit and changelog information):

```bash
./scripts/build.sh                  # native build and package
./scripts/build.sh --arch arm64     # ARM64 cross build for Orin
./scripts/build.sh --clean          # clean build
```

## Run

```bash
source install/setup.bash
ros2 launch teleop_description_moveit_config demo3.launch.py     # full stack
# headless:
xvfb-run -a ros2 launch teleop_description_moveit_config demo3.launch.py
```

Quick checks:

```bash
ros2 action list
ros2 topic echo /heart_beat
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

## Scripts

| Script | Purpose |
|--------|---------|
| `build.sh` | Unified build + packaging (native / Docker x86_64 / ARM64) |
| `cross_compile.sh` | x86_64 → ARM64 cross build and release package |
| `pack_install.sh` | Tar up `install/` |
| `release.sh` | Upload packages from `releases/` to the artifact repository; requires the `ARTIFACTORY_AUTH` env var (base64 `user:token`) |
| `run.sh` | Source and launch `demo3.launch.py` |
| `start_sdk_moveit.sh` | One-shot start on the robot: Cerebellum SDK → homing / mode switch → MoveIt, cleans up child processes on failure |
| `start_moveit_init_pose.sh` / `start_moveit_activate_and_go_init.sh` | Start MoveIt and move to the initial pose |
| `move_right_arm_loop.sh` | Right-arm motion loop test |
| `monitor_joint_topics.sh` | Record joint topics to a bag |
| `bag_to_csv.py` | Convert rosbag2 to CSV |
| `simulate_trajectory_pybullet.py` | Replay trajectories in PyBullet |

## Conventions

- All packages live under `src/`; `build/ install/ log/`, `*_compile_output/` and `releases/` are git-ignored.
- Interface changes go into `kernel_msgs` only, with dependants' `package.xml` updated accordingly.
- Credentials are injected through environment variables and never committed.
