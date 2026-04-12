#!/bin/bash
set -eo pipefail

# ============================================================
# 需要在实机HOME路径下执行
# 一键启动脚本：Cerebellum SDK -> Homing/模式切换 -> MoveIt
# 任何阶段失败，自动清理所有子进程
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_DIR="${SCRIPT_DIR}/linden_cerebellum_sdk"
MOVEIT_DIR="${SCRIPT_DIR}/linden_robot_moveit"

# 记录后台进程 PID
SDK_PID=""
MOVEIT_PID=""

SLEEP_INTERVAL=10

# --- 清理函数：终止所有子进程 ---
cleanup() {
    echo ""
    echo "[cleanup] 正在终止所有子进程..."

    if [[ -n "$MOVEIT_PID" ]] && kill -0 "$MOVEIT_PID" 2>/dev/null; then
        echo "[cleanup] 终止 MoveIt (PID=$MOVEIT_PID)"
        kill -TERM "$MOVEIT_PID" 2>/dev/null
        wait "$MOVEIT_PID" 2>/dev/null || true
    fi

    if [[ -n "$SDK_PID" ]] && kill -0 "$SDK_PID" 2>/dev/null; then
        echo "[cleanup] 终止 Cerebellum SDK (PID=$SDK_PID)"
        kill -TERM "$SDK_PID" 2>/dev/null
        wait "$SDK_PID" 2>/dev/null || true
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
# 阶段 1：启动 Cerebellum SDK（后台长期运行）
# ============================================================
echo "=========================================="
echo "[阶段1] 启动 Cerebellum SDK ..."
echo "=========================================="

cd "$SDK_DIR"
source install/setup.bash

bash ./scripts/run.sh &
SDK_PID=$!
echo "[阶段1] SDK 已启动 (PID=$SDK_PID)"

echo "[阶段1] 等待 ${SLEEP_INTERVAL} 秒让 SDK 初始化..."
sleep "$SLEEP_INTERVAL"

check_alive "$SDK_PID" "Cerebellum SDK"
echo "[阶段1] SDK 运行正常。"

# ============================================================
# 阶段 2：复位 + 切换到位置控制模式（一次性命令）
# ============================================================
echo ""
echo "=========================================="
echo "[阶段2] 执行 homing 和切换位置控制模式 ..."
echo "=========================================="

cd "$SDK_DIR"

echo "[阶段2] 执行 homing.sh ..."
bash ./tools/homing.sh
echo "[阶段2] homing 完成。"

echo "[阶段2] 等待 ${SLEEP_INTERVAL} 秒..."
sleep "$SLEEP_INTERVAL"

check_alive "$SDK_PID" "Cerebellum SDK"

echo "[阶段2] 切换到位置控制模式 (mode: 1) ..."
ros2 service call /cerebellum_sdk/arm/joint_mode \
    cerebellum_sdk_msg/srv/MotorMode "{mode: 1}"

if [[ $? -ne 0 ]]; then
    echo "[ERROR] 位置控制模式切换失败！"
    exit 1
fi
echo "[阶段2] 位置控制模式切换成功。"

echo "[阶段2] 等待 ${SLEEP_INTERVAL} 秒..."
sleep "$SLEEP_INTERVAL"

check_alive "$SDK_PID" "Cerebellum SDK"

# ============================================================
# 阶段 3：启动 MoveIt 运动规划（后台长期运行）
# ============================================================
echo ""
echo "=========================================="
echo "[阶段3] 启动 MoveIt 运动规划 ..."
echo "=========================================="

cd "$MOVEIT_DIR"
source install/setup.bash

xvfb-run -a ros2 launch teleop_description_moveit_config demo3.launch.py &
MOVEIT_PID=$!
echo "[阶段3] MoveIt 已启动 (PID=$MOVEIT_PID)"

echo "[阶段3] 等待 ${SLEEP_INTERVAL} 秒让 MoveIt 初始化..."
sleep "$SLEEP_INTERVAL"

check_alive "$MOVEIT_PID" "MoveIt"
echo "[阶段3] MoveIt 运行正常。"

# ============================================================
# 全部启动完成，持续监控
# ============================================================
echo ""
echo "=========================================="
echo "[完成] 所有服务已启动！"
echo "  SDK   PID = $SDK_PID"
echo "  MoveIt PID = $MOVEIT_PID"
echo "按 Ctrl+C 终止所有服务。"
echo "=========================================="

# 持续监控两个后台服务，任一退出则全部终止
while true; do
    check_alive "$SDK_PID" "Cerebellum SDK"
    check_alive "$MOVEIT_PID" "MoveIt"
    sleep 5
done
