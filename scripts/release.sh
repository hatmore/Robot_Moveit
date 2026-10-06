#!/bin/bash

# ============================================================
# 发布脚本：上传 releases 目录中的文件到 Artifactory
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
RELEASES_DIR="${WS_DIR}/releases"

# Artifactory 配置
ARTIFACTORY_URL="https://artifacts.iflytek.com/artifactory/LDJQR-private-repo/CI_BUILD/linden_robot_moveit"
# Basic Auth 令牌（base64 的 user:token），通过环境变量传入，禁止写死在仓库里
ARTIFACTORY_AUTH="${ARTIFACTORY_AUTH:-}"
if [ -z "${ARTIFACTORY_AUTH}" ]; then
    echo "[ERROR] 请先设置环境变量 ARTIFACTORY_AUTH（base64 的 user:token）" >&2
    exit 1
fi

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_error()   { echo -e "${RED}[ERROR]${NC} $1"; }

# 检查 releases 目录
if [ ! -d "${RELEASES_DIR}" ]; then
    print_error "releases 目录不存在: ${RELEASES_DIR}"
    exit 1
fi

# 统计文件数量
file_count=$(find "${RELEASES_DIR}" -type f -name "*.tar.gz" | wc -l)
if [ "$file_count" -eq 0 ]; then
    print_error "releases 目录中没有 .tar.gz 文件"
    exit 1
fi

print_info "找到 ${file_count} 个文件待上传"

# 上传文件
uploaded=0
failed=0

for file in "${RELEASES_DIR}"/*.tar.gz; do
    if [ -f "$file" ]; then
        filename=$(basename "$file")
        print_info "上传: ${filename}"

        http_code=$(curl -s -w "%{http_code}" -o /tmp/upload_response.json \
            -H "Authorization:Basic ${ARTIFACTORY_AUTH}" \
            -X PUT -T "$file" \
            "${ARTIFACTORY_URL}/${filename}")

        if [ "$http_code" -eq 200 ] || [ "$http_code" -eq 201 ]; then
            print_success "上传成功: ${filename}"
            uploaded=$((uploaded + 1))
        else
            print_error "上传失败: ${filename} (HTTP ${http_code})"
            cat /tmp/upload_response.json 2>/dev/null
            failed=$((failed + 1))
        fi
    fi
done

echo ""
echo "=========================================="
echo " 上传完成"
echo "=========================================="
echo "  成功: ${uploaded}"
echo "  失败: ${failed}"
echo "=========================================="

if [ "$failed" -gt 0 ]; then
    exit 1
fi