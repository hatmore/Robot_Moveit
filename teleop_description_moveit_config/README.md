# Teleop Description MoveIt Config

机器人 MoveIt2 配置包，包含多种启动模式。

## Launch 文件说明

### 主要启动文件

| 文件 | 描述 | 用途 |
|------|------|------|
| `demo.launch.py` | 基础 Demo | 最简单的 MoveIt 启动，仅包含基本功能 |
| `demo2.launch.py` | 关节/TCP 位置控制 | 启动位置控制服务器（OMPL 规划器） |
| `demo3.launch.py` | 全功能启动 | OMPL + Pilz 双规划器 + Servo + 位置控制 |
| `demo4.launch.py` | Pilz 轨迹规划 | 专注于 Pilz 工业运动规划器 |

### 功能对比

| 功能 | demo | demo2 | demo3 | demo4 |
|------|:----:|:-----:|:-----:|:-----:|
| MoveIt 基础 | ✓ | ✓ | ✓ | ✓ |
| RViz 可视化 | ✓ | ✓ | ✓ | ✓ |
| OMPL 规划器 | ✓ | ✓ | ✓ | - |
| Pilz 规划器 | - | ✓ | ✓ | ✓ |
| 关节位置控制 | - | ✓ | ✓ | - |
| TCP 位置控制 | - | ✓ | ✓ | - |
| Servo 实时控制 | - | - | ✓ | - |
| 轨迹规划 | - | - | ✓ | ✓ |

---

### 辅助启动文件

| 文件 | 描述 |
|------|------|
| `demo_twist.launch.py` | 双臂 Servo Twist 控制测试 |
| `test_servo.launch.py` | Servo 节点单独测试 |
| `demo_example.launch.py` | 轮式机械臂 Servo 示例 |
| `pilz_moveit.launch.py` | 纯 Pilz 规划器启动 |
| `fake_sim.py` | 仿真接口测试 |

### 基础组件启动文件

| 文件 | 描述 |
|------|------|
| `move_group.launch.py` | MoveGroup 节点 |
| `moveit_rviz.launch.py` | RViz 可视化 |
| `rsp.launch.py` | Robot State Publisher |
| `spawn_controllers.launch.py` | 控制器加载 |
| `static_virtual_joint_tfs.launch.py` | 静态 TF 发布 |
| `warehouse_db.launch.py` | MongoDB 数据库 |
| `setup_assistant.launch.py` | MoveIt Setup Assistant |

---

## 使用方法

### 基础启动
```bash
ros2 launch teleop_description_moveit_config demo.launch.py
```

### 位置控制模式
```bash
ros2 launch teleop_description_moveit_config demo2.launch.py
```

### 全功能模式（推荐）
```bash
ros2 launch teleop_description_moveit_config demo3.launch.py
```

### Pilz 轨迹规划模式
```bash
ros2 launch teleop_description_moveit_config demo4.launch.py
```

## 启动参数

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `use_rviz` | true | 是否启动 RViz |
| `db` | false | 是否启动 MongoDB |
| `debug` | false | 调试模式 |
| `enable_servo` | true | 启用 Servo 控制 (demo3) |
| `enable_left_arm_servo` | true | 启用左臂 Servo |
| `enable_right_arm_servo` | true | 启用右臂 Servo |

示例：
```bash
ros2 launch teleop_description_moveit_config demo3.launch.py use_rviz:=false
```
