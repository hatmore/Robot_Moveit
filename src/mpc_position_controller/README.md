# MPC Position Controller

Position-level Model Predictive Controller plugin for ros2_control,作为 `JointTrajectoryController` 的直插式替换。

## 原理

在每个控制周期求解一个带约束的二次规划 (QP) 问题，基于双积分器动力学模型预测未来 N 步状态，输出当前最优位置指令。

### 数学模型

**状态空间**（离散时间，dt = 1/update_rate Hz）：

```
状态:  x = [q; q_dot]     (关节位置 + 速度, 2n 维)
输入:  u = q_ddot          (关节加速度, n 维)

动力学:
       [q_{k+1}    ]   [I   dt*I] [q_k    ]   [0.5*dt^2*I]
       [            ] = [        ] [        ] + [           ] u_k
       [q_dot_{k+1}]   [0     I ] [q_dot_k]   [   dt*I    ]
```

**凝聚 QP 公式**：

将 N 步预测展开为：`X = S_x * x_0 + S_u * U`

其中 U = [u_0; u_1; ...; u_{N-1}]，然后求解：

```
min  U^T H U + g^T U
s.t. 加速度/速度/位置约束
```

- H = S_u^T Q S_u + R （对称正定，离线 Cholesky 分解）
- g = S_u^T Q (S_x x_0 - X_ref) （每周期在线计算）
- 求解方式：Cholesky 回代 + 逐关节约束投影
- 只执行 u_0，下一周期重新求解（滚动时域）

**n=7, N=10 时 QP 维度为 70x70，求解耗时 < 1us。**

### 与 JointTrajectoryController 的区别

| | JointTrajectoryController | MPCPositionController |
|---|---|---|
| 插值方式 | 五次样条 | MPC 最优控制 |
| 约束处理 | 无（依赖上游保证） | 位置/速度/加速度约束原生在 QP 中 |
| 预测能力 | 无 | N 步前瞻，提前感知约束 |
| 抗扰动 | 被动（等位置误差积累） | 主动（每周期重新优化） |
| 终端代价 | 无 | 可配置终端权重，提升稳定性 |

## 使用方法

### 1. 编译

```bash
colcon build --packages-select mpc_position_controller
```

### 2. 配置

在 `ros2_controllers.yaml` 中将控制器类型替换：

```yaml
controller_manager:
  ros__parameters:
    update_rate: 500

    left_arm_group_controller:
      type: mpc_position_controller/MPCPositionController  # 替换原 joint_trajectory_controller/JointTrajectoryController

left_arm_group_controller:
  ros__parameters:
    joints:
      - left_shoulder_pitch_joint
      - left_shoulder_roll_joint
      - left_shoulder_yaw_joint
      - left_elbow_joint
      - left_wrist_roll_joint
      - left_wrist_yaw_joint
      - left_wrist_pitch_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity

    # MPC 参数
    mpc:
      horizon: 10                    # 预测步数 N
      q_weight: 100.0                # 位置跟踪权重
      v_weight: 1.0                  # 速度跟踪权重
      r_weight: 0.01                 # 控制量（加速度）惩罚权重
      constraint_margin: 0.05        # 位置限位安全余量 [rad]
      terminal_cost_multiplier: 3.0  # 终端代价倍数

    # 关节约束
    limits:
      q_min: [-4.36332, -0.733038, -2.7925268, -2.338741, -2.8448867, -0.959931, -1.570796]
      q_max: [ 1.22173,  2.879793,  2.7925268,  2.338741,  2.8448867,  0.959931,  1.570796]
      v_max: [3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0]       # rad/s
      a_max: [9.0, 9.0, 9.0, 9.0, 12.0, 12.0, 12.0]     # rad/s^2
```

完整的左右臂示例配置见 `config/mpc_controllers.yaml`。

### 3. 接口兼容性

本控制器与 JointTrajectoryController 接口完全兼容，**现有系统无需其他改动**：

| 接口 | 说明 |
|---|---|
| `~/joint_trajectory` topic | MoveIt Servo 发布轨迹指令 |
| `~/follow_joint_trajectory` action | MoveIt MoveGroup 执行规划 |
| position command interface | 写入 ros2_control 硬件接口 |
| position + velocity state interface | 读取当前关节状态 |

## 参数说明

### MPC 参数 (`mpc.*`)

| 参数 | 默认值 | 说明 |
|---|---|---|
| `horizon` | 10 | 预测步数 N，越大越平滑但计算量增加 |
| `q_weight` | 100.0 | 位置跟踪权重，越大跟踪越紧 |
| `v_weight` | 1.0 | 速度跟踪权重 |
| `r_weight` | 0.01 | 加速度惩罚权重，越大运动越平滑但响应变慢 |
| `constraint_margin` | 0.05 | 位置限位内缩安全余量 [rad] |
| `terminal_cost_multiplier` | 3.0 | 最后一步代价倍率，提升终端稳定性 |

### 约束参数 (`limits.*`)

| 参数 | 说明 |
|---|---|
| `q_min` / `q_max` | 关节位置上下限 [rad]，应与 URDF 一致 |
| `v_max` | 关节速度限制 [rad/s] |
| `a_max` | 关节加速度限制 [rad/s^2] |

### 调参建议

- **跟踪更紧**：增大 `q_weight`（如 200~500）
- **运动更平滑**：增大 `r_weight`（如 0.1~1.0）或增大 `horizon`
- **快速响应**：减小 `r_weight`，减小 `horizon`
- **接近限位时更保守**：增大 `constraint_margin`

## 文件结构

```
mpc_position_controller/
├── include/mpc_position_controller/
│   ├── mpc_position_controller.hpp   # Controller plugin 定义
│   └── mpc_solver.hpp                # MPC 求解器定义
├── src/
│   ├── mpc_position_controller.cpp   # 生命周期、update()、action server
│   └── mpc_solver.cpp                # 凝聚 QP 构建与求解
├── config/
│   └── mpc_controllers.yaml          # 左右臂完整配置示例
├── mpc_position_controller_plugin.xml
├── CMakeLists.txt
└── package.xml
```

## 依赖

- ROS 2 Humble
- ros2_control / controller_interface
- Eigen3
- realtime_tools
