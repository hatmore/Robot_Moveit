# kernel_msgs

机器人各层之间的 ROS 2 接口定义（msg / srv / action），由其他功能包依赖，本身不含节点。

| 包 | 用途 |
|----|------|
| `planning_sdk_msgs` | 规划层对外接口：`TcpPosition` / `TcpPositionDelta` / `TcpTrajectory` / `JointPosition` / `JointPositionDelta` / `ExecuteTrajectory` 等 action，`SetJointLimits` / `GetJointLimits` / `RegisterFrame` 服务，`HeartBeat` / `TcpPose` / `TcpVelocityOnce` / `JointVelocityOnce` 等消息 |
| `cerebellum_sdk_msg` | 与底层 Cerebellum SDK 交互：`HeartBeat` / `MotorState` 消息，`MotorMode` / `AutoMode` / `GripperForce` / `LEDState` / `McuOta` 服务 |
| `power_borad_communication` | 电源板状态 `PowerBoardStatus` |

修改接口后需要重新编译依赖它们的包：

```bash
colcon build --packages-up-to basic_control_topic
```
