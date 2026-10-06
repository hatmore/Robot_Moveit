#!/bin/bash
set -eo pipefail

# ============================================================
# 需要在实机HOME路径下执行
# 一键启动脚本：Homing/模式切换 -> MoveIt
# 任何阶段失败，自动清理所有子进程
# ============================================================
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MOVEIT_DIR="$(dirname "$SCRIPT_DIR")"
source "$MOVEIT_DIR/../linden_cerebellum_sdk/install/setup.bash"

# 记录后台进程 PID
MOVEIT_PID=""

SLEEP_INTERVAL=15

# --- 清理函数：终止所有子进程 ---
cleanup() {
    echo ""
    echo "[cleanup] 正在终止所有子进程..."

    if [[ -n "$MOVEIT_PID" ]] && kill -0 "$MOVEIT_PID" 2>/dev/null; then
        echo "[cleanup] 终止 MoveIt 进程树 (PID=$MOVEIT_PID)"
        # 杀死整个进程树（父进程及所有子进程）
        pkill -TERM -P "$MOVEIT_PID" 2>/dev/null || true
        kill -TERM "$MOVEIT_PID" 2>/dev/null || true
        sleep 1
        # 强制杀死残留进程
        pkill -KILL -P "$MOVEIT_PID" 2>/dev/null || true
        kill -KILL "$MOVEIT_PID" 2>/dev/null || true
    fi

    echo "[cleanup] 清理完成。"
}

trap cleanup EXIT INT TERM

# --- 辅助：检查后台进程是否还活着 ---
check_alive() {
    local pid=$1
    local name=$2
    if ! kill -0 "$pid" 2>/dev/null; then
        echo "[ERROR] $name (PID=$pid) 已意外退出！"
        exit 1
    fi
}

# ============================================================
# 阶段 1：切换到位置控制模式
# ============================================================
echo ""
echo "=========================================="
echo "[阶段1] 切换到位置控制模式 ..."
echo "=========================================="

echo "[阶段1] 切换到位置控制模式 (mode: 1) ..."
ros2 service call /cerebellum_sdk/arm/joint_mode \
    cerebellum_sdk_msg/srv/MotorMode "{mode: 1}"

if [[ $? -ne 0 ]]; then
    echo "[ERROR] 位置控制模式切换失败！"
    exit 1
fi
echo "[阶段1] 位置控制模式切换成功。"

echo "[阶段1] 等待 ${SLEEP_INTERVAL} 秒..."
sleep 2

# ============================================================
# 阶段 2：启动 MoveIt 运动规划（后台长期运行）
# ============================================================
echo ""
echo "=========================================="
echo "[阶段2] 启动 MoveIt 运动规划 ..."
echo "=========================================="
source "$MOVEIT_DIR/install/setup.bash"

xvfb-run -a ros2 launch teleop_description_moveit_config demo3.launch.py &
MOVEIT_PID=$!
echo "[阶段2] MoveIt 已启动 (PID=$MOVEIT_PID)"

echo "[阶段2] 等待 ${SLEEP_INTERVAL} 秒让 MoveIt 初始化..."
sleep "$SLEEP_INTERVAL"

check_alive "$MOVEIT_PID" "MoveIt"
echo "[阶段2] MoveIt 运行正常。"

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
sleep 2
echo "[阶段3] 发送右臂初始姿态..."
ros2 action send_goal \
    /algorithm/grasp_planning/move/joint_position \
    planning_sdk_msgs/action/JointPosition \
    "$RIGHT_GOAL" --feedback
echo "[阶段3] 右臂移动完成。"

# ============================================================
# 全部启动完成，持续监控
# ============================================================
echo ""
echo "=========================================="
echo "[完成] 所有服务已启动！"
echo "  MoveIt PID = $MOVEIT_PID"
echo "按 Ctrl+C 终止所有服务。"
echo "=========================================="

# 持续监控 MoveIt
while true; do
    check_alive "$MOVEIT_PID" "MoveIt"
    sleep 5
done
