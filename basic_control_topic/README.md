# Basic Control Topic

ROS2 机械臂控制接口包，提供多种控制方式。

## 依赖

- ROS2 Humble
- MoveIt2
- planning_sdk_msgs

## 启动

所有功能都需要先启动 MoveIt：

```bash
ros2 launch teleop_description_moveit_config demo3.launch.py
或者
xvfb-run -a ros2 launch teleop_description_moveit_config demo3.launch.py
```

---

## 1. 验证实机系统状态

```bash
# 检查 action 列表
ros2 action list

# 检查控制器状态
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers

# 检查机器人关节
ros2 topic echo /cerebellum_sdk/arm/joint_commands
ros2 topic echo /cerebellum_sdk/arm/joint_states --once
```

---

## 2. Joint Position

关节位置控制。

```bash
ros2 run basic_control_topic joint_position_server
```

**左臂移动：**

```bash
ros2 action send_goal /algorithm/grasp_planning/move/joint_position \
  planning_sdk_msgs/action/JointPosition \
  "{timestamp: {sec: 0, nanosec: 0},
    target_state: {
      header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
      name: ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_yaw_joint', 'left_wrist_pitch_joint'],
      position: [0.7854, -0.6109, -0.3491, -1.5708, 0.2617, 0.0, -0.8727],
      velocity: [],
      effort: []
    }
  }" --feedback

# 左手门字第一个点
ros2 action send_goal /algorithm/grasp_planning/move/joint_position \
  planning_sdk_msgs/action/JointPosition \
  "{timestamp: {sec: 0, nanosec: 0},
    target_state: {
      header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
      name: ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_yaw_joint', 'left_wrist_pitch_joint'],
      position: [-0.232200,0.045240,-0.657192,-1.419835,0.123699,-0.245976,0.121161],
      velocity: [],
      effort: []
    }
  }" --feedback


**右臂移动**
ros2 action send_goal /algorithm/grasp_planning/move/joint_position \
  planning_sdk_msgs/action/JointPosition \
  "{timestamp: {sec: 0, nanosec: 0},
    target_state: {
      header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
      name: ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_yaw_joint', 'right_wrist_pitch_joint'],
      position: [-0.7854, 0.6109, 0.3491, 1.5708, -0.2617, 0.0, 0.8726],
      velocity: [],
      effort: []
    }
  }" --feedback

# 右手门字第一个点
ros2 action send_goal /algorithm/grasp_planning/move/joint_position \
  planning_sdk_msgs/action/JointPosition \
  "{timestamp: {sec: 0, nanosec: 0},
    target_state: {
      header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
      name: ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_yaw_joint', 'right_wrist_pitch_joint'],
      position: [0.174658,0.017840,0.680560,1.611908,-0.088732,0.298805,-0.234375],
      velocity: [],
      effort: []
    }
  }" --feedback
```

---

## 3. Joint Position Delta

关节增量控制。

**左臂增量移动：**

```bash
ros2 action send_goal /algorithm/grasp_planning/move/joint_position_delta planning_sdk_msgs/action/JointPositionDelta "{                                                                                             
    timestamp: {sec: 0, nanosec: 0},                                                                                                                                                                                 
    name: ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_elbow_joint'],
    velocity: 20,
    position_delta: [0.1, 0.0, 0.0]
  }"
```

---

## 4. TCP Position

末端位置控制。

**左臂移动：**

```bash
ros2 action send_goal /algorithm/grasp_planning/move/left_arm/tcp_position planning_sdk_msgs/action/TcpPosition "{
  rotation_style: 0,
  pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.4, y: 0.1, z: 0.7}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
}"
```

**右臂移动：**

```bash
ros2 action send_goal /algorithm/grasp_planning/move/right_arm/tcp_position planning_sdk_msgs/action/TcpPosition "{
  rotation_style: 0,
  pose: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.4, y: -0.3, z: 0.7}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
}"
```

---

## 5. TCP Position Delta

末端增量控制。

```bash
ros2 run basic_control_topic tcp_position_delta_action_server
```

**左臂增量移动：**

```bash
ros2 action send_goal /algorithm/grasp_planning/move/left_arm/tcp_position_delta planning_sdk_msgs/action/TcpPositionDelta "{
  rotation_style: 0,
  pose_delta: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.008726535498373935, y: 0.0, z: 0.0, w: 0.9999619230641713}}}
}"


0.008726535498373935,0.0,0.0,0.9999619230641713
```

**右臂增量移动：**

```bash
ros2 action send_goal /algorithm/grasp_planning/move/right_arm/tcp_position_delta planning_sdk_msgs/action/TcpPositionDelta "{
  rotation_style: 0,
  pose_delta: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.00, y: -0.2, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}
}"
```

---

## 6. TCP Velocity

末端速度控制。

```bash
ros2 topic pub -r 10 /algorithm/grasp_planning/move/left_arm/tcp_velocity_once   planning_sdk_msgs/msg/TcpVelocityOnce   "{timestamp: {sec: 0, nanosec: 0}, rotation_style: 0, velocity: {linear: {x: -80.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

ros2 topic pub -r 10 /algorithm/grasp_planning/move/left_arm/tcp_velocity_once   planning_sdk_msgs/msg/TcpVelocityOnce   "{timestamp: {sec: 0, nanosec: 0}, rotation_style: 0, velocity: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 40.0, y: 0.0, z: 0.0}}}"

ros2 topic pub -r 10 /algorithm/grasp_planning/move/left_arm/tcp_velocity_once   planning_sdk_msgs/msg/TcpVelocityOnce   "{timestamp: {sec: 0, nanosec: 0}, rotation_style: 0, velocity: {linear: {x: 0.0, y: -20.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"


# 往左
ros2 topic pub -r 10 /algorithm/grasp_planning/move/right_arm/tcp_velocity_once   planning_sdk_msgs/msg/TcpVelocityOnce   "{timestamp: {sec: 0, nanosec: 0}, rotation_style: 0, velocity: {linear: {x: 0.0, y: 70.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

# 往右
ros2 topic pub -r 10 /algorithm/grasp_planning/move/right_arm/tcp_velocity_once   planning_sdk_msgs/msg/TcpVelocityOnce   "{timestamp: {sec: 0, nanosec: 0}, rotation_style: 0, velocity: {linear: {x: 0.0, y: -70.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```

---

## 7. PTP 轨迹

点到点轨迹规划。

```bash
ros2 action send_goal /algorithm/grasp_planning/move/left_arm/tcp_trajectory planning_sdk_msgs/action/TcpTrajectory "{
    trajectory_file: '/home/pji/linden_robot_moveit/csv/test_left_v60.csv',
    tcp_actions: [{                                                                           
      action: 0,                                               
      target_point: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.048, y: 0.319, z: 0.415}, orientation: {x: 0.612, y: 0.354, z: -0.612, w: 0.354}}},
      velocity: 60,
      cont: 0                                     
    },             
    {           
      action: 0,
      target_point: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.048, y: 0.319, z: 0.515}, orientation: {x: 0.612, y: 0.354, z: -0.612, w: 0.354}}},
      velocity: 60,
      cont: 0
    },
    {
      action: 0,
      target_point: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.048, y: 0.319, z: 0.615}, orientation: {x: 0.612, y: 0.354, z: -0.612, w: 0.354}}},
      velocity: 60,
      cont: 0
    }
    ]
  }"

ros2 action send_goal /algorithm/grasp_planning/move/left_arm/execute_trajectory planning_sdk_msgs/action/ExecuteTrajectory "{                                                                                       
    trajectory_file: '/eibot/programs/35/1.csv',
    velocity: 50
}"

ros2 action send_goal /algorithm/grasp_planning/move/left_arm/tcp_trajectory planning_sdk_msgs/action/TcpTrajectory "{
    trajectory_file: '/home/pji/code/test_left_circ.csv',
    tcp_actions: [
    {           
      action: 2,
      target_point: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.342, y: 0.150, z: 0.681}, orientation: {x: 0.689, y: -0.159, z: -0.207, w: 0.676}}},
      velocity: 20,
      cont: 0
    },
    {                                                                           
      action: 0,                                               
      target_point: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.048, y: 0.319, z: 0.415}, orientation: {x: 0.612, y: 0.354, z: -0.612, w: 0.354}}},
      velocity: 40,
      cont: 0                                     
    },
    {                                                                           
      action: 1,                                               
      target_point: {header: {frame_id: 'base_link'}, pose: {position: {x: 0.048, y: 0.319, z: 0.515}, orientation: {x: 0.612, y: 0.354, z: -0.612, w: 0.354}}},
      velocity: 20,
      cont: 0                                     
    }
    ]
  }"

# 执行轨迹
ros2 action send_goal /algorithm/grasp_planning/move/left_arm/execute_trajectory planning_sdk_msgs/action/ExecuteTrajectory "{                                                                                       
    trajectory_file: '/home/pji/linden_robot_moveit/csv/test_left_circ.csv'                                                                                                                                            
}"

# 中断/继续轨迹
ros2 topic pub --once /algorithm/grasp_planning/move/trajectory_control std_msgs/msg/String "data: 'pause_left'" 
ros2 topic pub /algorithm/grasp_planning/move/trajectory_control std_msgs/msg/String "data: 'pause_left'" 
ros2 topic pub --once /algorithm/grasp_planning/move/trajectory_control std_msgs/msg/String "data: 'resume_left'" 
```

---

## 8. Joint Velocity

单关节速度控制。

```bash
ros2 topic pub -r 10 /algorithm/grasp_planning/move/joint_velocity_once planning_sdk_msgs/msg/JointVelocityOnce "{
  joint_name: 'left_shoulder_roll_joint',
  velocity: 30
}"
```

## 测试限位

````bash
# 查询所有关节限位（joint_names 空数组 = 全部）                                               
ros2 service call /algorithm/grasp_planning/get_joint_limits planning_sdk_msgs/srv/GetJointLimits "{joint_names: []}"                                                                       

# 查询单个关节
ros2 service call /algorithm/grasp_planning/get_joint_limits planning_sdk_msgs/srv/GetJointLimits "{joint_names: ['left_shoulder_roll_joint']}"

# 查看 topic 广播（格式变了，现在是 limits 数组）
ros2 topic echo /algorithm/joint_limits/current --once

# 改限位
ros2 service call /algorithm/grasp_planning/set_joint_limits planning_sdk_msgs/srv/SetJointLimits "{limits: [{joint_name: 'left_shoulder_roll_joint', lower_limit: -0.733, upper_limit: 0.733, velocity_limit: 3.0, effort_limit: 0.0}]}"
````

## 仿真模式
````bash
ros2 service call /algorithm/grasp_planning/simulator/mode std_srvs/srv/SetBool "{data: true}"

# 关闭仿真模式（切回真机）
ros2 service call /algorithm/grasp_planning/simulator/mode std_srvs/srv/SetBool "{data: false}"
````