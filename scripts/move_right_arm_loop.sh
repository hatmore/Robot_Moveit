#!/bin/bash

# 脚本名称：move_right_arm_loop.sh
# 功能：让右臂按照“往左 2s -> 停 1s -> 往右 1s”的模式循环运动

MOVEIT_DIR="$(dirname "$SCRIPT_DIR")"

# 参数定义 (单位：秒)
MOVE_LEFT_TIME=5
STOP_TIME=1
MOVE_RIGHT_TIME=5

# 消息发布频率 (Hz)
PUBLISH_RATE=10

# ROS 2 主题和消息类型
TOPIC="/algorithm/grasp_planning/move/right_arm/tcp_velocity_once"
MSG_TYPE="planning_sdk_msgs/msg/TcpVelocityOnce"

# 消息内容（JSON 格式）
MSG_LEFT='{
  "timestamp": {"sec": 0, "nanosec": 0},
  "rotation_style": 0,
  "velocity": {"linear": {"x": 0.0, "y": 80.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
}'

MSG_RIGHT='{
  "timestamp": {"sec": 0, "nanosec": 0},
  "rotation_style": 0,
  "velocity": {"linear": {"x": 0.0, "y": -80.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
}'

MSG_STOP='{
  "timestamp": {"sec": 0, "nanosec": 0},
  "rotation_style": 0,
  "velocity": {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
}'



# 函数：发送消息并等待指定时间
publish_and_wait() {
  local msg="$1"
  local duration="$2"
  # 计算需要发送的消息数量
  local count=$(echo "$duration * $PUBLISH_RATE" | bc -l | awk '{printf "%.0f", $1}')
  # 发送消息
  ros2 topic pub -r $PUBLISH_RATE $TOPIC $MSG_TYPE "$msg" &
  local pid=$!
  # 等待指定时间
  sleep $duration
  # 杀死发布进程
  kill $pid 2>/dev/null
}

source "$MOVEIT_DIR/install/setup.bash"

# 主循环
echo "开始右臂循环运动。按 Ctrl+C 停止。"
while true; do

  echo "等 ${STOP_TIME}s"
  sleep  $STOP_TIME

  echo "阶段 1：往左移动 ${MOVE_LEFT_TIME}s"
  publish_and_wait "$MSG_LEFT" $MOVE_LEFT_TIME

  echo "阶段 2：停止 ${STOP_TIME}s"
  sleep $STOP_TIME

  echo "阶段 3：往右移动 ${MOVE_RIGHT_TIME}s"
  publish_and_wait "$MSG_RIGHT" $MOVE_RIGHT_TIME

  echo "循环完成，开始下一周期..."
done