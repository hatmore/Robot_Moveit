#!/bin/bash
set -eo pipefail

# ============================================================
# 统一构建脚本：支持本地编译和 Docker 交叉编译
#
# 用法:
#   ./scripts/build.sh                    # 本地编译并打包
#   ./scripts/build.sh --arch x86_64      # x86_64 Docker 编译并打包
#   ./scripts/build.sh --arch arm64       # ARM64 Docker 交叉编译并打包
#   ./scripts/build.sh --clean            # 清理后编译
#   ./scripts/build.sh --pack-only        # 仅打包（需要先编译）
#
# 产物目录:
#   本地编译:
#     local_compile_output/                    (本地编译输出)
#       ├── build/
#       ├── install/
#       ├── log/
#       ├── scripts/
#       ├── BRANCH_INFO.txt                   (分支信息)
#       ├── COMMIT_INFO.txt                   (提交信息)
#       ├── CHANGELOG.txt                     (版本变更点)
#       └── build_info.json                   (JSON格式构建信息)
#   Docker 编译:
#     x86_compile_output/                      (x86_64 构建)
#       ├── build/
#       ├── install/
#       ├── log/
#       ├── scripts/
#       ├── BRANCH_INFO.txt                   (分支信息)
#       ├── COMMIT_INFO.txt                   (提交信息)
#       ├── CHANGELOG.txt                     (版本变更点)
#       └── build_info.json                   (JSON格式构建信息)
#     arm64_compile_output/                    (ARM64 构建)
#       ├── build/
#       ├── install/
#       ├── log/
#       ├── scripts/
#       ├── BRANCH_INFO.txt                   (分支信息)
#       ├── COMMIT_INFO.txt                   (提交信息)
#       ├── CHANGELOG.txt                     (版本变更点)
#       └── build_info.json                   (JSON格式构建信息)
#   releases/                                  (发布包，包名格式: linden_robot_moveit-${COMMIT_ID})
#     解压后结构:
#       ├── install/                          (ROS安装目录)
#       ├── scripts/                          (脚本目录)
#       ├── BRANCH_INFO.txt
#       ├── COMMIT_INFO.txt
#       ├── CHANGELOG.txt
#       └── build_info.json
# ============================================================

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_NAME="linden_robot_moveit"
ROS_DISTRO="humble"

# 默认配置
ARCH=""
BUILD_TYPE="Release"
CLEAN_BUILD=false
PACK_ONLY=false
PACKAGES=()
VERBOSE=false
NO_PACK=false  # 禁止打包选项

# 打印带颜色的消息
print_info()    { echo -e "${BLUE}[INFO]${NC} $1"; }
print_success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
print_warning() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
print_error()   { echo -e "${RED}[ERROR]${NC} $1"; }
print_step()    { echo -e "${CYAN}>>> $1${NC}"; }

# 打印使用说明
print_usage() {
    cat << EOF
用法: $0 [选项] [包名...]

选项:
  -a, --arch <arch>      目标架构: x86_64 | arm64（不指定则为本地编译）
  -t, --build-type <type> 构建类型: Release (默认) | Debug
  -c, --clean            清理后重新编译
  -n, --no-pack          编译后不打包（默认自动打包）
  -p, --pack-only        仅打包（需要先编译）
  --verbose              显示详细输出
  -h, --help             显示此帮助信息

示例:
  $0                              # 本地编译并打包
  $0 -a x86_64                    # x86_64 Docker 编译并打包
  $0 -a arm64                     # ARM64 Docker 交叉编译并打包
  $0 -n                           # 本地编译但不打包
  $0 -c                           # 清理后本地编译并打包
  $0 basic_pilz_motion_planning   # 仅编译指定包并打包
  $0 --pack-only                  # 仅打包（使用已有编译产物）

产物目录:
  本地编译:
    local_compile_output/         (本地编译输出)
      ├── build/
      ├── install/
      ├── log/
      ├── scripts/
      ├── BRANCH_INFO.txt         (分支信息)
      ├── COMMIT_INFO.txt         (提交信息)
      ├── CHANGELOG.txt           (版本变更点)
      └── build_info.json         (JSON格式构建信息)
  Docker 编译:
    x86_compile_output/           (x86_64)
      ├── build/
      ├── install/
      ├── log/
      ├── scripts/
      ├── BRANCH_INFO.txt         (分支信息)
      ├── COMMIT_INFO.txt         (提交信息)
      ├── CHANGELOG.txt           (版本变更点)
      └── build_info.json         (JSON格式构建信息)
    arm64_compile_output/         (ARM64)
      ├── build/
      ├── install/
      ├── log/
      ├── scripts/
      ├── BRANCH_INFO.txt         (分支信息)
      ├── COMMIT_INFO.txt         (提交信息)
      ├── CHANGELOG.txt           (版本变更点)
      └── build_info.json         (JSON格式构建信息)
  releases/                       (发布包，格式: linden_robot_moveit-\${COMMIT_ID})
    解压后结构:
      ├── install/                (ROS安装目录)
      ├── scripts/                (脚本目录)
      ├── BRANCH_INFO.txt
      ├── COMMIT_INFO.txt
      ├── CHANGELOG.txt
      └── build_info.json

Docker 镜像:
  x86_64: artifacts.iflytek.com/ldjqr-docker-release/ldtyjqrsyn/ptp-x86-build:v1.0
  arm64:  artifacts.iflytek.com/ldjqr-docker-release/ldtyjqrsyn/ptp-arm64-build:latest
EOF
}

# 解析命令行参数
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -a|--arch)
                ARCH="$2"
                if [[ -n "$ARCH" && "$ARCH" != "x86_64" && "$ARCH" != "arm64" ]]; then
                    print_error "不支持的架构: $ARCH (支持: x86_64, arm64，留空为本地编译)"
                    exit 1
                fi
                shift 2
                ;;
            -t|--build-type)
                BUILD_TYPE="$2"
                shift 2
                ;;
            -c|--clean)
                CLEAN_BUILD=true
                shift
                ;;
            -n|--no-pack)
                NO_PACK=true
                shift
                ;;
            -p|--pack-only)
                PACK_ONLY=true
                shift
                ;;
            --verbose)
                VERBOSE=true
                shift
                ;;
            -h|--help)
                print_usage
                exit 0
                ;;
            -*)
                print_error "未知选项: $1"
                print_usage
                exit 1
                ;;
            *)
                PACKAGES+=("$1")
                shift
                ;;
        esac
    done
}

# 检查 Docker 是否可用
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker 未安装，ARM64 交叉编译需要 Docker"
        exit 1
    fi

    if ! docker info &> /dev/null; then
        print_error "Docker 未运行"
        exit 1
    fi
}

# 清理构建目录
clean_build_dirs() {
    local arch=$1

    # 保存 releases 目录
    if [ -d "${WS_DIR}/releases" ]; then
        mv "${WS_DIR}/releases" "${WS_DIR}/.releases_backup"
    fi

    # 本地编译清理
    if [ -z "$arch" ]; then
        print_info "清理本地构建目录..."
        rm -rf "${WS_DIR}/build" 2>/dev/null || true
        rm -rf "${WS_DIR}/install" 2>/dev/null || true
        rm -rf "${WS_DIR}/log" 2>/dev/null || true
        rm -rf "${WS_DIR}/local_compile_output" 2>/dev/null || true
    fi

    # 清理 x86_64 输出目录
    if [ "${arch}" = "x86_64" ]; then
        print_info "清理 x86_64 构建目录..."
        rm -rf "${WS_DIR}/x86_compile_output" 2>/dev/null || true
    fi

    # 清理 ARM64 输出目录
    if [ "${arch}" = "arm64" ]; then
        print_info "清理 ARM64 构建目录..."
        rm -rf "${WS_DIR}/arm64_compile_output" 2>/dev/null || true
    fi

    # 恢复 releases 目录
    if [ -d "${WS_DIR}/.releases_backup" ]; then
        mv "${WS_DIR}/.releases_backup" "${WS_DIR}/releases"
    fi

    print_success "清理完成"
}

# 本地编译
build_local() {
    print_step "开始本地编译..."

    cd "${WS_DIR}"

    # 设置构建目录
    local build_dir="${WS_DIR}/build"
    local install_dir="${WS_DIR}/install"
    local log_dir="${WS_DIR}/log"
    local output_dir="${WS_DIR}/local_compile_output"

    # Source ROS 环境
    if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
        source /opt/ros/${ROS_DISTRO}/setup.bash
    else
        print_error "ROS2 ${ROS_DISTRO} 环境未找到"
        print_info "请在 ROS2 环境中运行此脚本，或使用 -a x86_64/arm64 进行 Docker 编译"
        exit 1
    fi

    # 构建参数
    local cmake_args="-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"

    local colcon_args="--symlink-install"

    if [ "$VERBOSE" = true ]; then
        colcon_args+=" --event-handlers console_direct+"
    fi

    # 指定包编译
    if [ ${#PACKAGES[@]} -gt 0 ]; then
        colcon_args+=" --packages-select ${PACKAGES[*]}"
        print_info "编译指定包: ${PACKAGES[*]}"
    fi

    # 执行编译
    print_info "执行 colcon build..."
    colcon build ${colcon_args} --cmake-args ${cmake_args}

    # 验证编译结果
    if [ -f "${install_dir}/setup.bash" ]; then
        print_success "本地编译成功!"
        print_info "安装目录: ${install_dir}"

        # 复制到 local_compile_output 目录
        print_info "复制编译产物到 local_compile_output..."
        mkdir -p "${output_dir}"
        cp -r "${build_dir}" "${output_dir}/"
        cp -r "${install_dir}" "${output_dir}/"
        cp -r "${log_dir}" "${output_dir}/"

        # 复制 scripts 目录到输出目录
        print_info "复制 scripts 目录..."
        cp -r "${WS_DIR}/scripts" "${output_dir}/scripts"
        chmod +x "${output_dir}/scripts/"*.sh 2>/dev/null || true

        # 生成构建信息（分支、commit、变更点）
        generate_build_info "${output_dir}"

        print_info "输出目录: ${output_dir}"
        print_info "  ├── build/"
        print_info "  ├── install/"
        print_info "  ├── log/"
        print_info "  ├── scripts/"
        print_info "  ├── BRANCH_INFO.txt"
        print_info "  ├── COMMIT_INFO.txt"
        print_info "  ├── CHANGELOG.txt"
        print_info "  └── build_info.json"
    else
        print_error "编译失败"
        exit 1
    fi
}

# x86_64 Docker 编译
build_x86_64() {
    print_step "开始 x86_64 Docker 编译..."

    check_docker

    # x86_64 构建镜像
    local x86_image="artifacts.iflytek.com/ldjqr-docker-release/ldtyjqrsyn/ptp-x86-build:v1.0"

    # 检查镜像是否存在
    if ! docker image inspect "${x86_image}" &> /dev/null; then
        print_info "拉取 x86_64 构建镜像: ${x86_image}"
        docker pull "${x86_image}"
    fi

    cd "${WS_DIR}"

    # 设置构建输出目录
    local output_dir="${WS_DIR}/x86_compile_output"
    local build_dir="${output_dir}/build"
    local install_dir="${output_dir}/install"
    local log_dir="${output_dir}/log"

    # 创建输出目录
    mkdir -p "${output_dir}"

    # 构建参数
    local cmake_args="-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    cmake_args+=" -DCMAKE_SKIP_INSTALL_RPATH=ON"
    cmake_args+=" -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
    cmake_args+=" -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON"

    local colcon_args="--build-base /workspace/x86_compile_output/build"
    colcon_args+=" --install-base /workspace/x86_compile_output/install"

    # 指定包编译
    local packages_args=""
    if [ ${#PACKAGES[@]} -gt 0 ]; then
        packages_args="--packages-select ${PACKAGES[*]}"
        print_info "编译指定包: ${PACKAGES[*]}"
    fi

    # 执行 Docker 编译
    print_info "执行 Docker 编译..."
    docker run --rm \
        --platform linux/amd64 \
        -v "${WS_DIR}:/workspace" \
        -w /workspace \
        -e ROS_DISTRO="${ROS_DISTRO}" \
        "${x86_image}" \
        bash -c "
            set -e
            source /opt/ros/${ROS_DISTRO}/setup.bash
            echo '>>> 开始 colcon build (x86_64)...'
            colcon build \
                ${colcon_args} \
                ${packages_args} \
                --event-handlers console_direct+ \
                --cmake-args ${cmake_args}
            echo '>>> 编译完成!'
        "

    # 验证编译结果
    if [ -f "${install_dir}/setup.bash" ]; then
        print_success "x86_64 编译成功!"
        print_info "安装目录: ${install_dir}"

        # 复制 scripts 目录到输出目录
        print_info "复制 scripts 目录..."
        cp -r "${WS_DIR}/scripts" "${output_dir}/scripts"
        chmod +x "${output_dir}/scripts/"*.sh 2>/dev/null || true

        # 生成构建信息（分支、commit、变更点）
        generate_build_info "${output_dir}"

        print_info "输出目录: ${output_dir}"
        print_info "  ├── build/"
        print_info "  ├── install/"
        print_info "  ├── log/"
        print_info "  ├── scripts/"
        print_info "  ├── BRANCH_INFO.txt"
        print_info "  ├── COMMIT_INFO.txt"
        print_info "  ├── CHANGELOG.txt"
        print_info "  └── build_info.json"
    else
        print_error "编译失败"
        exit 1
    fi
}

# ARM64 交叉编译（带重试机制）
build_arm64() {
    print_step "开始 ARM64 交叉编译..."

    check_docker

    # ARM64 构建镜像
    local arm64_image="artifacts.iflytek.com/ldjqr-docker-release/ldtyjqrsyn/ptp-arm64-build:v1.0"

    # 检查镜像是否存在
    if ! docker image inspect "${arm64_image}" &> /dev/null; then
        print_info "拉取 ARM64 构建镜像: ${arm64_image}"
        docker pull "${arm64_image}"
    fi

    cd "${WS_DIR}"

    # 设置构建输出目录
    local output_dir="${WS_DIR}/arm64_compile_output"
    local build_dir="${output_dir}/build"
    local install_dir="${output_dir}/install"
    local log_dir="${output_dir}/log"

    # 创建输出目录
    mkdir -p "${output_dir}"

    # 配置 QEMU
    print_info "配置 QEMU 支持..."
    docker run --rm --privileged multiarch/qemu-user-static --reset -p yes &> /dev/null || true

    # 构建参数 - 限制并行度以提高 QEMU 稳定性
    local cmake_args="-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    cmake_args+=" -DCMAKE_SKIP_INSTALL_RPATH=ON"
    cmake_args+=" -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"
    cmake_args+=" -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=ON"
    # 限制 CMake 内部编译并行，减少 QEMU 内存压力
    cmake_args+=" -DCMAKE_JOB_POOL_COMPILE=compile_pool"
    cmake_args+=" -DCMAKE_JOB_POOL_LINK=link_pool"
    cmake_args+=" -DCMAKE_JOB_POOLS=\"compile_pool=2;link_pool=1\""

    local colcon_args="--build-base /workspace/arm64_compile_output/build"
    colcon_args+=" --install-base /workspace/arm64_compile_output/install"

    # 指定包编译
    local packages_args=""
    if [ ${#PACKAGES[@]} -gt 0 ]; then
        packages_args="--packages-select ${PACKAGES[*]}"
        print_info "编译指定包: ${PACKAGES[*]}"
    fi

    # 重试机制
    local max_retries=5
    local retry_count=0
    local build_success=false

    while [ $retry_count -lt $max_retries ]; do
        retry_count=$((retry_count + 1))
        print_info "执行 Docker 交叉编译 (尝试 $retry_count/$max_retries)..."

        if docker run --rm \
            --platform linux/arm64 \
            --memory=8g \
            --memory-swap=16g \
            -v "${WS_DIR}:/workspace" \
            -w /workspace \
            -e ROS_DISTRO="${ROS_DISTRO}" \
            "${arm64_image}" \
            bash -c "
                set -e
                source /opt/ros/${ROS_DISTRO}/setup.bash
                echo '>>> 开始 colcon build (ARM64)...'
                colcon build \
                    ${colcon_args} \
                    ${packages_args} \
                    --event-handlers console_direct+ \
                    --parallel-workers 1 \
                    --cmake-args ${cmake_args}
                echo '>>> 编译完成!'
            "; then
            build_success=true
            break
        else
            print_warning "编译失败 (尝试 $retry_count/$max_retries)"
            if [ $retry_count -lt $max_retries ]; then
                print_info "等待 5 秒后重试..."
                sleep 5
                # 重新配置 QEMU
                docker run --rm --privileged multiarch/qemu-user-static --reset -p yes &> /dev/null || true
            fi
        fi
    done

    if [ "$build_success" = false ]; then
        print_error "交叉编译失败，已重试 $max_retries 次"
        exit 1
    fi

    # 验证编译结果
    if [ -f "${install_dir}/setup.bash" ]; then
        print_success "ARM64 交叉编译成功!"
        print_info "安装目录: ${install_dir}"

        # 复制 scripts 目录到输出目录
        print_info "复制 scripts 目录..."
        cp -r "${WS_DIR}/scripts" "${output_dir}/scripts"
        chmod +x "${output_dir}/scripts/"*.sh 2>/dev/null || true

        # 生成构建信息（分支、commit、变更点）
        generate_build_info "${output_dir}"

        print_info "输出目录: ${output_dir}"
        print_info "  ├── build/"
        print_info "  ├── install/"
        print_info "  ├── log/"
        print_info "  ├── scripts/"
        print_info "  ├── BRANCH_INFO.txt"
        print_info "  ├── COMMIT_INFO.txt"
        print_info "  ├── CHANGELOG.txt"
        print_info "  └── build_info.json"
    else
        print_error "交叉编译失败"
        exit 1
    fi
}

# 创建发布包
create_package() {
    local arch=$1

    print_step "创建发布包..."

    # 获取 8 位 commit id 作为版本标识
    local commit_id=$(git rev-parse --short=8 HEAD 2>/dev/null || echo "unknown")

    # 根据架构确定 install 目录
    local install_dir
    local old_prefix
    local arch_name
    local output_dir

    if [ -z "$arch" ]; then
        # 本地编译
        install_dir="${WS_DIR}/local_compile_output/install"
        old_prefix="${WS_DIR}/local_compile_output/install"
        arch_name="$(uname -m)"
        output_dir="${WS_DIR}/local_compile_output"
    elif [ "${arch}" = "x86_64" ]; then
        install_dir="${WS_DIR}/x86_compile_output/install"
        old_prefix="/workspace/x86_compile_output/install"
        arch_name="x86_64"
        output_dir="${WS_DIR}/x86_compile_output"
    else
        install_dir="${WS_DIR}/arm64_compile_output/install"
        old_prefix="/workspace/arm64_compile_output/install"
        arch_name="arm64"
        output_dir="${WS_DIR}/arm64_compile_output"
    fi

    local release_dir="${WS_DIR}/releases"
    local package_name="${PROJECT_NAME}-${commit_id}-${arch_name}"
    local tar_name="${package_name}.tar.gz"
    local tar_path="${release_dir}/${tar_name}"
    local staging_dir="${release_dir}/.staging_${package_name}"

    if [ ! -d "${install_dir}" ]; then
        print_error "安装目录不存在: ${install_dir}"
        if [ -z "$arch" ]; then
            print_info "请先执行编译: $0"
        else
            print_info "请先执行编译: $0 --arch ${arch}"
        fi
        exit 1
    fi

    print_info "创建发布包: ${tar_name}"

    mkdir -p "${staging_dir}/install"
    mkdir -p "${release_dir}"

    # 复制 install 目录
    print_info "复制 install 目录..."
    cp -r "${install_dir}"/. "${staging_dir}/install/"

    # 复制 scripts 目录到 install 同级目录
    print_info "复制 scripts 目录..."
    mkdir -p "${staging_dir}/scripts"
    cp -r "${WS_DIR}/scripts/"* "${staging_dir}/scripts/" 2>/dev/null || true
    cp "${WS_DIR}/scripts/start_moveit_activate_and_go_init.sh" "${staging_dir}/scripts/run.sh" 2>/dev/null || true
    chmod +x "${staging_dir}/scripts/"*.sh 2>/dev/null || true

    # 替换硬编码路径
    patch_for_relocation "${staging_dir}/install" "${old_prefix}"

    # 复制构建信息文件（分支、commit、变更点）到 install 同级目录
    print_info "复制构建信息文件..."
    for info_file in BRANCH_INFO.txt COMMIT_INFO.txt CHANGELOG.txt build_info.json; do
        if [ -f "${output_dir}/${info_file}" ]; then
            cp "${output_dir}/${info_file}" "${staging_dir}/"
        fi
    done

    # 创建辅助文件（放在 install 目录内）
    create_version_info "${commit_id}" "${arch_name}" "${staging_dir}/install"
    create_launch_script "${staging_dir}/install"
    create_install_readme "${commit_id}" "${arch_name}" "${staging_dir}/install"

    # 打包（包含 install、scripts 和构建信息文件）
    print_info "打包为 tar.gz..."
    tar -czf "${tar_path}" -C "${staging_dir}" \
        install \
        scripts \
        BRANCH_INFO.txt \
        COMMIT_INFO.txt \
        CHANGELOG.txt \
        build_info.json

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
}

# 替换硬编码路径为占位符
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
    print_success "路径替换完成"
}

# 创建版本信息文件
create_version_info() {
    local commit_id=$1
    local arch=$2
    local output_dir=$3

    print_info "创建版本信息文件..."

    # 获取 Git 信息
    local git_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'N/A')
    local git_commit=$(git rev-parse HEAD 2>/dev/null || echo 'N/A')
    local git_commit_short=$(git rev-parse --short HEAD 2>/dev/null || echo 'N/A')
    local git_describe=$(git describe --tags --always 2>/dev/null || echo 'N/A')
    local git_remote=$(git remote get-url origin 2>/dev/null || echo 'N/A')

    cat > "${output_dir}/VERSION_INFO.txt" << EOF
================================================================================
${PROJECT_NAME} 版本信息
================================================================================
Commit ID:   ${commit_id}
目标架构:    ${arch}
构建时间:    $(date '+%Y-%m-%d %H:%M:%S')
构建主机:    $(hostname)
Git 分支:    ${git_branch}
Git Commit:  ${git_commit_short}
Git 状态:    ${git_describe}

================================================================================
EOF

    cat > "${output_dir}/version.json" << EOF
{
  "project": "${PROJECT_NAME}",
  "commit_id": "${commit_id}",
  "arch": "${arch}",
  "build_time": "$(date -u '+%Y-%m-%dT%H:%M:%SZ')",
  "build_host": "$(hostname)",
  "git_branch": "${git_branch}",
  "git_commit": "${git_commit}",
  "git_commit_short": "${git_commit_short}",
  "git_remote": "${git_remote}"
}
EOF
}

# 生成构建信息文件（分支、commit、变更点）
generate_build_info() {
    local output_dir=$1

    print_info "生成构建信息文件..."

    mkdir -p "${output_dir}"

    # 获取 Git 信息
    local git_branch=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo 'N/A')
    local git_commit=$(git rev-parse HEAD 2>/dev/null || echo 'N/A')
    local git_commit_short=$(git rev-parse --short HEAD 2>/dev/null || echo 'N/A')
    local git_author=$(git log -1 --format='%an <%ae>' 2>/dev/null || echo 'N/A')
    local git_date=$(git log -1 --format='%ci' 2>/dev/null || echo 'N/A')
    local git_message=$(git log -1 --format='%s' 2>/dev/null || echo 'N/A')
    local git_remote=$(git remote get-url origin 2>/dev/null || echo 'N/A')

    # 生成分支信息文件
    cat > "${output_dir}/BRANCH_INFO.txt" << EOF
================================================================================
${PROJECT_NAME} 分支信息
================================================================================
当前分支:   ${git_branch}
远程仓库:   ${git_remote}
构建时间:   $(date '+%Y-%m-%d %H:%M:%S')

分支最后提交:
  Author:  ${git_author}
  Date:    ${git_date}
  Message: ${git_message}

================================================================================
EOF

    # 生成 COMMIT 信息文件
    print_info "生成 COMMIT 信息..."
    cat > "${output_dir}/COMMIT_INFO.txt" << EOF
================================================================================
${PROJECT_NAME} COMMIT 信息
================================================================================
最新 Commit:
  Hash:       ${git_commit}
  Short:      ${git_commit_short}
  Author:     ${git_author}
  Date:       ${git_date}
  Message:    ${git_message}

================================================================================
最近 20 条提交记录:
================================================================================
EOF

    # 添加最近 20 条提交记录
    git log -20 --pretty=format:"%h - %an, %ar : %s" 2>/dev/null >> "${output_dir}/COMMIT_INFO.txt" || echo "无法获取提交历史" >> "${output_dir}/COMMIT_INFO.txt"

    echo "" >> "${output_dir}/COMMIT_INFO.txt"
    echo "================================================================================" >> "${output_dir}/COMMIT_INFO.txt"

    # 生成版本变更点信息文件
    print_info "生成版本变更点信息..."
    local changelog_file="${output_dir}/CHANGELOG.txt"

    cat > "${changelog_file}" << EOF
================================================================================
${PROJECT_NAME} 版本变更点
================================================================================
生成时间: $(date '+%Y-%m-%d %H:%M:%S')
当前分支: ${git_branch}
当前提交: ${git_commit_short}

================================================================================
近期变更记录 (最近 30 条):
================================================================================

EOF

    # 获取详细的变更记录
    git log -30 --pretty=format:"----------------------------------------
Commit:  %h
Author:  %an <%ae>
Date:    %ci
Message: %s

Files changed:" 2>/dev/null >> "${changelog_file}" || echo "无法获取变更记录" >> "${changelog_file}"

    # 添加每个提交的文件变更统计
    echo "" >> "${changelog_file}"

    # 使用更简洁的格式添加变更统计
    git log -30 --pretty=format:"%h|%an|%ar|%s" --stat 2>/dev/null | while IFS='|' read -r hash author date msg; do
        if [[ -n "$hash" && "$hash" != *"|"* ]]; then
            echo "" >> "${changelog_file}"
            echo "[$hash] $msg" >> "${changelog_file}"
            echo "  Author: $author, $date" >> "${changelog_file}"
        fi
    done

    # 添加文件变更统计摘要
    cat >> "${changelog_file}" << EOF


================================================================================
文件变更统计 (相对于最近一次 tag):
================================================================================

EOF

    # 尝试找到最近的 tag
    local last_tag=$(git describe --tags --abbrev=0 2>/dev/null)
    if [ -n "$last_tag" ]; then
        echo "上次发布版本: ${last_tag}" >> "${changelog_file}"
        echo "" >> "${changelog_file}"
        echo "提交统计:" >> "${changelog_file}"
        git log ${last_tag}..HEAD --oneline 2>/dev/null | wc -l | xargs -I {} echo "  自 ${last_tag} 以来共有 {} 次提交" >> "${changelog_file}"
        echo "" >> "${changelog_file}"
        echo "变更文件:" >> "${changelog_file}"
        git diff --stat ${last_tag}..HEAD 2>/dev/null >> "${changelog_file}" || echo "  无变更" >> "${changelog_file}"
    else
        echo "未找到最近的 tag，显示最近 10 次提交的文件变更:" >> "${changelog_file}"
        echo "" >> "${changelog_file}"
        git log -10 --name-status --oneline 2>/dev/null >> "${changelog_file}" || echo "  无法获取变更" >> "${changelog_file}"
    fi

    echo "" >> "${changelog_file}"
    echo "================================================================================" >> "${changelog_file}"

    # 生成 JSON 格式的构建信息（便于程序读取）
    cat > "${output_dir}/build_info.json" << EOF
{
  "project": "${PROJECT_NAME}",
  "build_time": "$(date -u '+%Y-%m-%dT%H:%M:%SZ')",
  "git": {
    "branch": "${git_branch}",
    "commit": "${git_commit}",
    "commit_short": "${git_commit_short}",
    "author": "${git_author}",
    "date": "${git_date}",
    "message": "${git_message}",
    "remote": "${git_remote}"
  }
}
EOF

    print_success "构建信息文件生成完成"
    print_info "  - BRANCH_INFO.txt  (分支信息)"
    print_info "  - COMMIT_INFO.txt  (提交信息)"
    print_info "  - CHANGELOG.txt    (版本变更点)"
    print_info "  - build_info.json  (JSON格式构建信息)"
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
    local commit_id=$1
    local arch=$2
    local package_dir=$3

    cat > "${package_dir}/INSTALL_README.md" << EOF
# ${PROJECT_NAME} (${commit_id}, ${arch}) 安装说明

## 系统要求

- ROS2 Humble
- Ubuntu 22.04 (${arch})
- 依赖包: moveit, ros2_control

## 安装步骤

### 1. 解压

\`\`\`bash
mkdir -p ~/ros2_ws
cd ~/ros2_ws
tar -xzf ${PROJECT_NAME}-${commit_id}-${arch}.tar.gz
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

详见以下文件:
- VERSION_INFO.txt  版本基本信息
- version.json      JSON 格式版本信息
- BRANCH_INFO.txt   分支信息
- COMMIT_INFO.txt   提交信息
- CHANGELOG.txt     版本变更点
- build_info.json   JSON 格式构建信息
EOF
}

# 打印构建摘要
print_build_summary() {
    echo ""
    echo "=========================================="
    echo " 构建摘要"
    echo "=========================================="

    if [ -z "${ARCH}" ]; then
        echo "  编译模式:   本地编译"
        echo "  构建类型:   ${BUILD_TYPE}"
        echo "  工作目录:   ${WS_DIR}"
        echo ""

        local output_dir="${WS_DIR}/local_compile_output"
        local install_dir="${output_dir}/install"

        if [ -d "${install_dir}" ]; then
            echo "  产物目录:"
            echo "    local_compile_output/"
            echo "      ├── build/"
            echo "      ├── install/"
            echo "      ├── log/"
            echo "      ├── scripts/"
            echo "      ├── BRANCH_INFO.txt"
            echo "      ├── COMMIT_INFO.txt"
            echo "      ├── CHANGELOG.txt"
            echo "      └── build_info.json"
            echo ""

            local pkg_count=$(find "${install_dir}" -maxdepth 1 -type d | wc -l)
            pkg_count=$((pkg_count - 1))

            echo "  已安装包:   ${pkg_count} 个"
            echo ""
        fi
    else
        echo "  目标架构:   ${ARCH}"
        echo "  编译模式:   Docker 编译"
        echo "  构建类型:   ${BUILD_TYPE}"
        echo "  工作目录:   ${WS_DIR}"
        echo ""

        local output_dir="${WS_DIR}/${ARCH}_compile_output"
        local install_dir="${output_dir}/install"

        if [ -d "${install_dir}" ]; then
            echo "  产物目录:"
            echo "    ${ARCH}_compile_output/"
            echo "      ├── build/"
            echo "      ├── install/"
            echo "      ├── log/"
            echo "      ├── scripts/"
            echo "      ├── BRANCH_INFO.txt"
            echo "      ├── COMMIT_INFO.txt"
            echo "      ├── CHANGELOG.txt"
            echo "      └── build_info.json"
            echo ""

            local pkg_count=$(find "${install_dir}" -maxdepth 1 -type d | wc -l)
            pkg_count=$((pkg_count - 1))

            echo "  已安装包:   ${pkg_count} 个"
            echo ""
        fi
    fi

    echo "  发布包:     ${WS_DIR}/releases/"
    echo "=========================================="
    echo ""
}

# 主函数
main() {
    parse_args "$@"

    echo ""
    echo "=========================================="
    echo " ${PROJECT_NAME} 构建脚本"
    echo "=========================================="
    echo ""

    # 仅打包模式
    if [ "$PACK_ONLY" = true ]; then
        create_package "${ARCH}"
        print_build_summary
        exit 0
    fi

    # 清理
    if [ "$CLEAN_BUILD" = true ]; then
        clean_build_dirs "${ARCH}"
    fi

    # 执行编译
    if [ -z "${ARCH}" ]; then
        # 本地编译
        build_local
    else
        # Docker 编译
        case "${ARCH}" in
            x86_64)
                build_x86_64
                ;;
            arm64)
                build_arm64
                ;;
        esac
    fi

    # 默认打包（除非指定了 --no-pack）
    if [ "$NO_PACK" != true ]; then
        create_package "${ARCH}"
    fi

    print_build_summary
}

main "$@"