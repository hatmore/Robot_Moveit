#!/bin/bash
# 稳定版：仅录包，无echo崩溃问题
set -e

# 加载ROS环境
source /opt/ros/humble/setup.bash
source install/setup.bash

# 创建日志目录（修复1970时间戳问题）
LOG_DIR="./joint_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
BAG_PATH="$LOG_DIR/joint_bag"

echo "================================="
echo " 关节话题录包（稳定版，无崩溃）"
echo " 日志目录: $LOG_DIR"
echo " 按 Ctrl+C 停止录制"
echo "================================="

# 仅保留rosbag录包，删除崩溃的ros2 topic echo
# 录制你急停实验需要的核心话题
ros2 bag record \
  /joint_states \
  /cerebellum_sdk/arm/joint_states \
  /cerebellum_sdk/arm/joint_commands \
  -o "$BAG_PATH"

echo "录包完成！文件保存在: $BAG_PATH"