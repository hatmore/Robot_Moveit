#!/bin/bash
set -eo pipefail

# ============================================================
# 打包 install 目录
# 用法:
#   ./scripts/pack_install.sh                # 默认输出带时间戳的 tar.gz
#   ./scripts/pack_install.sh output.tar.gz  # 指定输出文件名
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT="${1:-${WS_DIR}/linden_robot_moveit_${TIMESTAMP}.tar.gz}"

cd "$WS_DIR"

if [[ ! -d "install" ]]; then
    echo "[pack] install 目录不存在，先编译..."
    colcon build --symlink-install
fi

echo "[pack] 打包 install 目录..."
tar -czf "$OUTPUT" install/

SIZE=$(du -h "$OUTPUT" | cut -f1)
echo "[pack] 完成: $OUTPUT ($SIZE)"
