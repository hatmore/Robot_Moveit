# Robot_Moveit

双臂机器人 **ROS 2 Humble + MoveIt 2** 控制工作空间。在 MoveIt 之上封装了一层稳定的 ROS 接口（action / service / topic），供上层任务系统下发预设位姿、关节/末端移动、轨迹执行、末端速度伺服等任务，并提供模块心跳监控、左右臂独立调度、仿真/实机接口切换等系统能力。规划器同时支持 **OMPL**、**Pilz 工业运动规划器** 与 **MoveIt Servo**。

## 目录结构

```
Robot_Moveit/                          # colcon 工作空间根目录
├── src/
│   ├── basic_control_topic/           # 核心控制层：对外 action / service / topic 节点
│   ├── basic_pilz_motion_planning/    # Pilz PTP / LIN / CIRC / 序列规划示例
│   ├── kernel_msgs/                   # 接口定义
│   │   ├── planning_sdk_msgs/         #   规划层对外接口（action / srv / msg）
│   │   ├── cerebellum_sdk_msg/        #   与底层 Cerebellum SDK 的接口
│   │   └── power_borad_communication/ #   电源板状态
│   ├── mpc_position_controller/       # ros2_control 位置级 MPC 控制器插件
│   └── teleop_description_moveit_config/  # MoveIt 配置包（URDF/SRDF、控制器、launch）
├── scripts/                           # 构建、打包、发布、现场启动脚本
├── .gitignore
└── README.md
```

### 功能包

| 包 | 说明 |
|----|------|
| `basic_control_topic` | `basic_control_node` 组合节点 + 一组独立节点：`joint_position_server`、`joint_position_delta_server`、`tcp_position_action_server`、`tcp_position_delta_action_server`、`tcp_trajectory_action_server`、`execute_trajectory_server`、`joint_velocity_to_servo`、`tcp_velocity_to_servo`、`servo_manager`、`tcp_pose_publisher`、`joint_limits_server`、`frame_registration_server/client`、`heart_beat`、`sim_interface_switch`、`initial_joint_positions`。详见 [包内 README](src/basic_control_topic/README.md) |
| `basic_pilz_motion_planning` | `pilz_ptp_joint`、`pilz_ptp_cartesian`、`pilz_lin_joint`、`pilz_lin_cartesian`、`pilz_circ_cartesian`、`pilz_sequence_demo` 等示例程序 |
| `kernel_msgs/*` | 仅包含接口定义，见 [kernel_msgs/README.md](src/kernel_msgs/README.md) |
| `mpc_position_controller` | 可直接替换 `JointTrajectoryController` 的 MPC 控制器，原理与参数见 [包内 README](src/mpc_position_controller/README.md) |
| `teleop_description_moveit_config` | `demo.launch.py`（基础）、`demo2.launch.py`（位置控制）、`demo3.launch.py`（全功能：OMPL + Pilz + Servo）、`demo4.launch.py`（Pilz 轨迹）等，launch 对比见 [包内 README](src/teleop_description_moveit_config/README.md) |

## 环境依赖

- Ubuntu 22.04，ROS 2 **Humble**
- MoveIt 2（`ros-humble-moveit`、`ros-humble-moveit-servo`）
- ros2_control / ros2_controllers
- 其余依赖通过 `rosdep` 安装：

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
rosdep install --from-paths src --ignore-src -r -y
```

## 构建

```bash
git clone https://github.com/hatmore/Robot_Moveit.git
cd Robot_Moveit
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

或使用统一构建脚本（本地 / Docker x86_64 / Docker ARM64，产物自动带分支、提交、变更记录）：

```bash
./scripts/build.sh                  # 本地编译并打包
./scripts/build.sh --arch arm64     # 面向 Orin 的 ARM64 交叉编译
./scripts/build.sh --clean          # 清理后编译
```

## 运行

```bash
source install/setup.bash
ros2 launch teleop_description_moveit_config demo3.launch.py     # 全功能启动
# 无显示环境：
xvfb-run -a ros2 launch teleop_description_moveit_config demo3.launch.py
```

快速验证：

```bash
ros2 action list
ros2 topic echo /heart_beat
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

## 脚本

| 脚本 | 用途 |
|------|------|
| `build.sh` | 统一构建 + 打包（本地 / Docker x86_64 / ARM64） |
| `cross_compile.sh` | x86_64 → ARM64 交叉编译并生成发布包 |
| `pack_install.sh` | 把 `install/` 打成 tar.gz |
| `release.sh` | 把 `releases/` 下的发布包上传到制品库，需要环境变量 `ARTIFACTORY_AUTH`（base64 的 `user:token`） |
| `run.sh` | source 后启动 `demo3.launch.py` |
| `start_sdk_moveit.sh` | 实机一键启动：Cerebellum SDK → 回零/模式切换 → MoveIt，异常自动清理子进程 |
| `start_moveit_init_pose.sh` / `start_moveit_activate_and_go_init.sh` | 启动 MoveIt 并回到初始位姿 |
| `move_right_arm_loop.sh` | 右臂循环运动测试 |
| `monitor_joint_topics.sh` | 关节话题录包 |
| `bag_to_csv.py` | rosbag2 转 CSV |
| `simulate_trajectory_pybullet.py` | 在 PyBullet 中回放轨迹 |

## 开发约定

- 所有功能包放在 `src/` 下，`build/ install/ log/` 与各类 `*_compile_output/`、`releases/` 已被 `.gitignore` 忽略。
- 对外接口变更只改 `kernel_msgs`，并同步更新依赖方的 `package.xml`。
- 凭据一律通过环境变量注入，禁止写入仓库。
