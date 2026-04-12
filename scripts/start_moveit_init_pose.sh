#!/bin/bash
set -eo pipefail

# ============================================================
# 单纯启动 MoveIt，完全就绪后将双臂移动到初始姿态
# 在 linden_robot_moveit 目录下执行
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MOVEIT_DIR="${SCRIPT_DIR}"

MOVEIT_PID=""
SLEEP_INTERVAL=10

# --- 清理函数 ---
cleanup() {
    echo ""
    echo "[cleanup] 正在终止 MoveIt..."
    if [[ -n "$MOVEIT_PID" ]] && kill -0 "$MOVEIT_PID" 2>/dev/null; then
        kill -TERM "$MOVEIT_PID" 2>/dev/null
        wait "$MOVEIT_PID" 2>/dev/null || true
    fi
    echo "[cleanup] 清理完成。"
}

trap cleanup EXIT INT TERM

# --- 检查进程是否存活 ---
check_alive() {
    local pid=$1
    local name=$2
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[ERROR] $name (PID=$pid) 已意外退出！"
        exit 1
    fi
}

# ============================================================
# 阶段 1：启动 MoveIt（后台运行）
# ============================================================
echo "=========================================="
echo "[阶段1] 启动 MoveIt 运动规划 ..."
echo "=========================================="

cd "$MOVEIT_DIR"
source install/setup.bash

xvfb-run -a ros2 launch teleop_description_moveit_config demo3.launch.py &
MOVEIT_PID=$!
echo "[阶段1] MoveIt 已启动 (PID=$MOVEIT_PID)"

echo "[阶段1] 等待 ${SLEEP_INTERVAL} 秒让 MoveIt 初始化..."
sleep "$SLEEP_INTERVAL"

check_alive "$MOVEIT_PID" "MoveIt"
echo "[阶段1] MoveIt 运行正常。"

# ============================================================
# 阶段 2：等待 move_group 完全就绪
# ============================================================
echo ""
echo "=========================================="
echo "[阶段2] 等待 move_group 完全就绪 ..."
echo "=========================================="

WAIT_TIMEOUT=60
ELAPSED=0
while ! ros2 node list 2>/dev/null | grep -q "move_group"; do
    if [[ $ELAPSED -ge $WAIT_TIMEOUT ]]; then
        echo "[ERROR] 等待 move_group 超时（${WAIT_TIMEOUT}s）！"
        exit 1
    fi
    echo "[阶段2] move_group 尚未就绪，继续等待..."
    sleep 3
    ELAPSED=$((ELAPSED + 3))
    check_alive "$MOVEIT_PID" "MoveIt"
done

echo "[阶段2] move_group 已就绪，额外等待 ${SLEEP_INTERVAL} 秒确保控制器完全加载..."
sleep "$SLEEP_INTERVAL"

check_alive "$MOVEIT_PID" "MoveIt"

# ============================================================
# 阶段 3：将双臂移动到初始姿态
# ============================================================
echo ""
echo "=========================================="
echo "[阶段3] 将双臂移动到初始姿态 ..."
echo "=========================================="

LEFT_GOAL="{timestamp: {sec: 0, nanosec: 0}, target_state: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, name: ['left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_yaw_joint', 'left_wrist_pitch_joint'], position: [0.7854, -0.6109, -0.3491, -1.5708, 0.2617, 0.0, -0.8727], velocity: [], effort: []}}"

RIGHT_GOAL="{timestamp: {sec: 0, nanosec: 0}, target_state: {header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'}, name: ['right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_yaw_joint', 'right_wrist_pitch_joint'], position: [-0.7854, 0.6109, 0.3491, 1.5708, -0.2617, 0.0, 0.8726], velocity: [], effort: []}}"

echo "[阶段3] 发送左臂初始姿态..."
ros2 action send_goal \
    /algorithm/grasp_planning/move/joint_position \
    planning_sdk_msgs/action/JointPosition \
    "$LEFT_GOAL" --feedback
echo "[阶段3] 左臂移动完成。"

check_alive "$MOVEIT_PID" "MoveIt"
sleep 3
echo "[阶段3] 发送右臂初始姿态..."
ros2 action send_goal \
    /algorithm/grasp_planning/move/joint_position \
    planning_sdk_msgs/action/JointPosition \
    "$RIGHT_GOAL" --feedback
echo "[阶段3] 右臂移动完成。"

# ============================================================
# 持续监控 MoveIt
# ============================================================
echo ""
echo "=========================================="
echo "[完成] 双臂已到达初始姿态，MoveIt 持续运行中。"
echo "  MoveIt PID = $MOVEIT_PID"
echo "按 Ctrl+C 终止服务。"
echo "=========================================="

while true; do
    check_alive "$MOVEIT_PID" "MoveIt"
    sleep 5
done
