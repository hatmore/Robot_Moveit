#!/bin/bash
set -eo pipefail

# ============================================================
# 交叉编译 + 发布脚本：x86_64 -> ARM64 (Orin)
# 使用 Docker + QEMU 模拟 ARM64 环境
#
# 用法:
#   ./scripts/cross_compile.sh <版本号> [选项]
#   例如: ./scripts/cross_compile.sh 1.0.0
#         ./scripts/cross_compile.sh 1.0.0 --build-only
#         ./scripts/cross_compile.sh 1.0.0 --pack-only
#
# 环境变量（可选）:
#   ARTIFACT_UPLOAD_TYPE - 上传类型: scp|curl|aws|none (默认: none)
#   ARTIFACT_REPO_URL    - 制品仓库地址
#   ARTIFACT_REPO_AUTH   - Basic Auth 令牌
# ============================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_NAME="linden_robot_moveit"

# 配置
ROS_DISTRO="humble"
DOCKER_IMAGE="ros:${ROS_DISTRO}"
RELEASE_DIR="${WS_DIR}/releases"
ARCH="arm64"

# 上传配置
ARTIFACT_UPLOAD_TYPE="${ARTIFACT_UPLOAD_TYPE:-none}"
ARTIFACT_REPO_URL="${ARTIFACT_REPO_URL:-}"
ARTIFACT_REPO_AUTH="${ARTIFACT_REPO_AUTH:-}"
ARTIFACT_REPO_USER="${ARTIFACT_REPO_USER:-}"
ARTIFACT_REPO_PASS="${ARTIFACT_REPO_PASS:-}"

# 打印带颜色的消息
print_info() { echo -e "${BLUE}[INFO]${NC} $1" >&2; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1" >&2; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1" >&2; }
print_error() { echo -e "${RED}[ERROR]${NC} $1" >&2; }

# 打印使用说明
print_usage() {
    cat << EOF
用法: $0 <版本号> [选项]

参数:
  版本号          发布版本号，例如: 1.0.0, 1.0.0-beta.1

选项:
  -h, --help      显示此帮助信息
  --build-only    仅编译，不打包
  --pack-only     仅打包（需要先编译）
  --upload-type   指定上传类型: scp|curl|aws|none (默认: none)
  --repo-url      指定制品仓库地址

示例:
  $0 1.0.0
  $0 1.0.0 --upload-type curl --repo-url https://repo.example.com/artifacts

环境变量:
  ARTIFACT_REPO_URL    - 制品仓库地址
  ARTIFACT_REPO_AUTH   - Basic Auth 令牌 (base64编码)
  ARTIFACT_UPLOAD_TYPE - 上传类型
EOF
}

# 验证版本号格式
validate_version() {
    local version=$1
    if [[ ! $version =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[a-zA-Z0-9._-]+)?$ ]]; then
        print_error "版本号格式错误: $version"
        print_error "正确格式: x.x.x 或 x.x.x-suffix，例如: 1.0.0, 1.0.0-beta.1"
        exit 1
    fi
}

# 清理构建目录
clean_build() {
    print_info "清理构建目录..."
    cd "${WS_DIR}"

    # 保存 releases 目录
    if [ -d "${RELEASE_DIR}" ]; then
        mv "${RELEASE_DIR}" "${WS_DIR}/.releases_backup"
    fi

    # 清理 build 和 install
    rm -rf build install log 2>/dev/null || true

    # 恢复 releases 目录
    if [ -d ".releases_backup" ]; then
        mv "${WS_DIR}/.releases_backup" "${RELEASE_DIR}"
    fi

    print_success "清理完成"
}

# 创建版本信息文件
create_version_info() {
    local version=$1
    local arch=$2
    local output_dir=$3

    print_info "创建版本信息文件..."

    cat > "${output_dir}/VERSION_INFO.txt" << EOF
================================================================================
${PROJECT_NAME} 版本信息
================================================================================
版本号:     ${version}
目标架构:   ${arch} (Orin)
构建时间:   $(date '+%Y-%m-%d %H:%M:%S')
构建主机:   $(hostname)
Git 分支:   $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'N/A')
Git Commit: $(git rev-parse --short HEAD 2>/dev/null || echo 'N/A')
Git 状态:   $(git describe --tags --always 2>/dev/null || echo 'N/A')

================================================================================
EOF

    cat > "${output_dir}/version.json" << EOF
{
  "project": "${PROJECT_NAME}",
  "version": "${version}",
  "arch": "${arch}",
  "build_time": "$(date -u '+%Y-%m-%dT%H:%M:%SZ')",
  "build_host": "$(hostname)",
  "git_branch": "$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'N/A')",
  "git_commit": "$(git rev-parse --short HEAD 2>/dev/null || echo 'N/A')"
}
EOF

    print_success "版本信息文件创建完成"
}

# 将 install 目录内所有文件的硬编码路径替换为占位符
patch_for_relocation() {
    local staging_install="$1"
    local old_prefix="$2"
    local placeholder="__COLCON_INSTALL_PREFIX__"

    print_info "替换硬编码路径为占位符（可重定位打包）..."

    find "${staging_install}" -type f \( \
        -name "*.bash" -o -name "*.sh"  -o \
        -name "*.cmake" -o -name "*.dsv" -o \
        -name "*.pc"    -o -name "*.pth" \
    \) | while read -r f; do
        if grep -qF "${old_prefix}" "$f" 2>/dev/null; then
            sed -i "s|${old_prefix}|${placeholder}|g" "$f"
        fi
    done

    find "${staging_install}" -type f ! -name "*.*" | while read -r f; do
        if file "$f" 2>/dev/null | grep -q "text" && grep -qF "${old_prefix}" "$f" 2>/dev/null; then
            sed -i "s|${old_prefix}|${placeholder}|g" "$f"
        fi
    done

    cat > "${staging_install}/relocate.sh" << 'RELOCATE_EOF'
#!/bin/bash
# =============================================================
# 重定位脚本：在目标机器上解压后执行一次即可
# 用法：bash install/relocate.sh
# =============================================================
set -e

INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLACEHOLDER="__COLCON_INSTALL_PREFIX__"
MARKER="${INSTALL_DIR}/.relocated"

if [ -f "${MARKER}" ]; then
    echo "[relocate] 已经重定位过，跳过。"
    exit 0
fi

echo "[relocate] 目标路径: ${INSTALL_DIR}"
echo "[relocate] 正在替换占位符..."

count=0
while IFS= read -r -d '' f; do
    if grep -qF "${PLACEHOLDER}" "$f" 2>/dev/null; then
        sed -i "s|${PLACEHOLDER}|${INSTALL_DIR}|g" "$f"
        count=$((count + 1))
    fi
done < <(find "${INSTALL_DIR}" -type f \( \
    -name "*.bash" -o -name "*.sh"  -o \
    -name "*.cmake" -o -name "*.dsv" -o \
    -name "*.pc"    -o -name "*.pth" \
\) -print0)

touch "${MARKER}"
echo "[relocate] 完成，共处理 ${count} 个文件。"
echo "[relocate] 现在可以执行: source ${INSTALL_DIR}/setup.bash"
RELOCATE_EOF

    chmod +x "${staging_install}/relocate.sh"
    print_success "路径替换完成，relocate.sh 已生成"
}

# 创建启动脚本
create_launch_script() {
    local package_dir=$1

    cat > "${package_dir}/start_moveit.sh" << 'EOF'
#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || true
source "${SCRIPT_DIR}/setup.bash"
echo "启动 Linden Robot MoveIt..."
ros2 launch teleop_description_moveit_config demo3.launch.py "$@"
EOF
    chmod +x "${package_dir}/start_moveit.sh"

    cat > "${package_dir}/setup_env.sh" << 'EOF'
#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash 2>/dev/null || true
source "${SCRIPT_DIR}/setup.bash"
echo "Linden Robot MoveIt 环境已加载"
EOF
    chmod +x "${package_dir}/setup_env.sh"
}

# 创建安装说明
create_install_readme() {
    local version=$1
    local arch=$2
    local package_dir=$3

    cat > "${package_dir}/INSTALL_README.md" << EOF
# ${PROJECT_NAME} v${version} (${arch}) 安装说明

## 系统要求

- ROS2 Humble
- Ubuntu 22.04 (ARM64)
- 依赖包: moveit, ros2_control

## 安装步骤

### 1. 解压

\`\`\`bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
tar -xzf ${PROJECT_NAME}_v${version}_${arch}.tar.gz
\`\`\`

### 2. 重定位（首次部署执行一次）

\`\`\`bash
bash ~/ros2_ws/install/relocate.sh
\`\`\`

### 3. 设置环境

\`\`\`bash
source ~/ros2_ws/install/setup.bash
\`\`\`

### 4. 启动

\`\`\`bash
./install/start_moveit.sh
# 或
ros2 launch teleop_description_moveit_config demo3.launch.py
\`\`\`

## 版本信息

详见 VERSION_INFO.txt 和 version.json
EOF
}

# 创建发布包
create_package() {
    local version=$1
    local package_name="${PROJECT_NAME}_v${version}_${ARCH}"
    local tar_name="${package_name}.tar.gz"
    local tar_path="${RELEASE_DIR}/${tar_name}"
    local staging_dir="${RELEASE_DIR}/.staging_${package_name}"

    print_info "创建发布包: ${tar_name}"

    mkdir -p "${staging_dir}/install"

    print_info "复制 install 目录..."
    cp -r "${WS_DIR}/install"/. "${staging_dir}/install/"

    print_info "复制运维脚本..."
    mkdir -p "${staging_dir}/install/scripts"
    cp "${WS_DIR}/scripts/monitor_joint_topics.sh" "${staging_dir}/install/scripts/" 2>/dev/null || true
    chmod +x "${staging_dir}/install/scripts/"*.sh 2>/dev/null || true

    # 替换硬编码路径
    patch_for_relocation "${staging_dir}/install" "/workspace/install"

    # 创建辅助文件
    create_version_info "${version}" "${ARCH}" "${staging_dir}/install"
    create_launch_script "${staging_dir}/install"
    create_install_readme "${version}" "${ARCH}" "${staging_dir}/install"

    # 打包
    print_info "打包为 tar.gz..."
    mkdir -p "${RELEASE_DIR}"
    tar -czf "${tar_path}" -C "${staging_dir}" install

    # 计算哈希
    local sha256_hash=$(sha256sum "${tar_path}" | awk '{print $1}')
    local file_size=$(du -h "${tar_path}" | cut -f1)

    cat > "${tar_path}.sha256" << EOF
SHA256 (${tar_name}) = ${sha256_hash}
SIZE = ${file_size}
EOF

    rm -rf "${staging_dir}"

    print_success "发布包创建成功"
    print_info "文件位置: ${tar_path}"
    print_info "文件大小: ${file_size}"
    print_info "SHA256:   ${sha256_hash}"

    echo "${tar_path}"
}

# 上传到制品仓库
upload_artifact() {
    local file_path=$1
    local version=$2
    local filename=$(basename "${file_path}")

    print_info "准备上传制品..."
    print_info "上传类型: ${ARTIFACT_UPLOAD_TYPE}"

    case "${ARTIFACT_UPLOAD_TYPE}" in
        scp)
            if [ -z "${ARTIFACT_REPO_URL}" ]; then
                print_error "未设置 ARTIFACT_REPO_URL"
                return 1
            fi
            print_info "通过 SCP 上传到: ${ARTIFACT_REPO_URL}"
            local remote_dir="${ARTIFACT_REPO_URL}/${version}"
            scp "${file_path}" "${remote_dir}/$(basename ${file_path})" || {
                local ssh_host=$(echo "${ARTIFACT_REPO_URL}" | cut -d':' -f1)
                local base_dir=$(echo "${ARTIFACT_REPO_URL}" | cut -d':' -f2)
                ssh "${ssh_host}" "mkdir -p ${base_dir}/${version}"
                scp "${file_path}" "${remote_dir}/$(basename ${file_path})"
            }
            scp "${file_path}.sha256" "${remote_dir}/$(basename ${file_path}).sha256" || true
            print_success "SCP 上传成功"
            ;;
        curl)
            if [ -z "${ARTIFACT_REPO_URL}" ]; then
                print_error "未设置 ARTIFACT_REPO_URL"
                return 1
            fi
            local upload_url="${ARTIFACT_REPO_URL}/${filename}"
            print_info "通过 CURL 上传到: ${upload_url}"
            cd "$(dirname "${file_path}")"
            if [ -n "${ARTIFACT_REPO_AUTH}" ]; then
                curl -s --progress-bar -H "Authorization:Basic ${ARTIFACT_REPO_AUTH}" -T "${filename}" "${upload_url}"
                curl -s --progress-bar -H "Authorization:Basic ${ARTIFACT_REPO_AUTH}" -T "${filename}.sha256" "${upload_url}.sha256" || true
            elif [ -n "${ARTIFACT_REPO_USER}" ] && [ -n "${ARTIFACT_REPO_PASS}" ]; then
                curl -s --progress-bar -u "${ARTIFACT_REPO_USER}:${ARTIFACT_REPO_PASS}" -T "${filename}" "${upload_url}"
                curl -s --progress-bar -u "${ARTIFACT_REPO_USER}:${ARTIFACT_REPO_PASS}" -T "${filename}.sha256" "${upload_url}.sha256" || true
            else
                curl -s --progress-bar -T "${filename}" "${upload_url}"
                curl -s --progress-bar -T "${filename}.sha256" "${upload_url}.sha256" || true
            fi
            print_success "CURL 上传成功"
            ;;
        aws)
            if [ -z "${ARTIFACT_REPO_URL}" ]; then
                print_error "未设置 ARTIFACT_REPO_URL (S3 bucket)"
                return 1
            fi
            local s3_key="${PROJECT_NAME}/${version}/${filename}"
            print_info "通过 AWS CLI 上传到 S3: ${ARTIFACT_REPO_URL}"
            aws s3 cp "${file_path}" "s3://${ARTIFACT_REPO_URL}/${s3_key}"
            aws s3 cp "${file_path}.sha256" "s3://${ARTIFACT_REPO_URL}/${s3_key}.sha256" || true
            print_success "AWS S3 上传成功"
            ;;
        none)
            print_warning "未配置上传方式，跳过上传"
            print_info "制品已保存至: ${file_path}"
            ;;
        *)
            print_error "未知的上传类型: ${ARTIFACT_UPLOAD_TYPE}"
            return 1
            ;;
    esac
}

# 主函数
main() {
    local version=""
    local build_only=false
    local pack_only=false

    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                print_usage
                exit 0
                ;;
            --build-only)
                build_only=true
                shift
                ;;
            --pack-only)
                pack_only=true
                shift
                ;;
            --upload-type)
                ARTIFACT_UPLOAD_TYPE="$2"
                shift 2
                ;;
            --repo-url)
                ARTIFACT_REPO_URL="$2"
                shift 2
                ;;
            -*)
                print_error "未知选项: $1"
                print_usage
                exit 1
                ;;
            *)
                if [ -z "$version" ]; then
                    version="$1"
                else
                    print_error "多余的参数: $1"
                    exit 1
                fi
                shift
                ;;
        esac
    done

    if [ -z "$version" ]; then
        print_error "请指定版本号"
        print_usage
        exit 1
    fi

    validate_version "$version"

    echo "=========================================="
    echo "交叉编译 + 发布配置"
    echo "=========================================="
    echo "版本号:     ${version}"
    echo "目标架构:   ${ARCH} (Orin)"
    echo "ROS 版本:   ${ROS_DISTRO}"
    echo "Docker 镜像: ${DOCKER_IMAGE}"
    echo "上传方式:   ${ARTIFACT_UPLOAD_TYPE}"
    echo "=========================================="

    # 检查 Docker
    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装"
        exit 1
    fi

    if ! docker info &> /dev/null; then
        print_error "Docker 未运行"
        exit 1
    fi

    # 编译
    if [ "$pack_only" = false ]; then
        echo ""
        echo "[1/3] 配置 QEMU 支持..."
        docker run --rm --privileged multiarch/qemu-user-static --reset -p yes &> /dev/null || true
        print_success "QEMU 配置完成"

        clean_build

        echo ""
        echo "[2/3] 开始交叉编译 (${ARCH})..."
        cd "${WS_DIR}"

        docker run --rm \
            --platform "linux/${ARCH}" \
            -v "${WS_DIR}:/workspace" \
            -w /workspace \
            -e ROS_DISTRO="${ROS_DISTRO}" \
            "${DOCKER_IMAGE}" \
            bash -c "
                set -e
                echo '>>> 安装编译依赖...'
                apt-get update -qq
                apt-get install -y -qq python3-colcon-common-extensions python3-pip > /dev/null

                echo '>>> Source ROS 环境...'
                source /opt/ros/${ROS_DISTRO}/setup.bash

                echo '>>> 开始 colcon build (可重定位模式)...'
                colcon build \
                    --cmake-args \
                    -DCMAKE_BUILD_TYPE=Release \
                    -DCMAKE_SKIP_INSTALL_RPATH=ON \
                    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON \
                    -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON

                echo '>>> 编译完成!'
            "

        print_success "交叉编译完成"
    fi

    # 打包
    if [ "$build_only" = false ]; then
        echo ""
        echo "[3/3] 创建发布包..."

        if [ ! -d "${WS_DIR}/install" ]; then
            print_error "install 目录不存在，请先编译"
            exit 1
        fi

        local package_path
        package_path=$(create_package "${version}")

        upload_artifact "${package_path}" "${version}"

        echo ""
        echo "=========================================="
        print_success "交叉编译 + 发布完成!"
        echo "=========================================="
        echo ""
        echo "  版本号:    ${version}"
        echo "  目标架构:  ${ARCH}"
        echo "  包文件:    ${package_path}"
        echo "  校验文件:  ${package_path}.sha256"
        echo ""
        echo "部署到 Orin:"
        echo "  1. scp ${package_path} orin@<IP>:~/"
        echo "  2. mkdir -p ~/ros2_ws && cd ~/ros2_ws"
        echo "  3. tar -xzf ~/$(basename ${package_path})"
        echo "  4. bash install/relocate.sh"
        echo "  5. source install/setup.bash"
        echo "=========================================="
    fi
}

main "$@"