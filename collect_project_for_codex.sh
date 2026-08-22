#!/usr/bin/env bash
set -Eeuo pipefail

workspace_root="$(pwd -P)"
workspace_name="$(basename "$workspace_root")"

# 可通过第一个参数指定输出目录
# 例如：./collect_project_for_codex.sh /mnt/shared
output_dir="${1:-"$(dirname "$workspace_root")"}"

timestamp="$(date +%Y%m%d_%H%M%S)"
archive_name="${workspace_name}_codex_${timestamp}.tar.gz"

mkdir -p "$output_dir"
output_dir="$(cd "$output_dir" && pwd -P)"
archive_path="${output_dir}/${archive_name}"

if [[ ! -d "${workspace_root}/src" ]]; then
    echo "错误：当前目录不是标准 ROS 工作区，找不到 src/"
    echo "请先进入工作区根目录，例如："
    echo "cd ~/terrain_radiation_ws"
    exit 1
fi

staging_dir="$(mktemp -d -t codex_project_XXXXXX)"

cleanup() {
    rm -rf "$staging_dir"
}

trap cleanup EXIT

mkdir -p "${staging_dir}/project"

echo "[1/5] 复制项目源码和配置..."

exclude_args=(
    --exclude='./build'
    --exclude='./install'
    --exclude='./log'
    --exclude='*/build'
    --exclude='*/install'
    --exclude='*/log'

    --exclude='.git'
    --exclude='*/.git/*'

    --exclude='__pycache__'
    --exclude='*/__pycache__/*'
    --exclude='.pytest_cache'
    --exclude='*/.pytest_cache/*'
    --exclude='*.pyc'
    --exclude='*.pyo'

    --exclude='*.bag'
    --exclude='*.bag.active'
    --exclude='*.db3'
    --exclude='*.mcap'

    --exclude='*.mp4'
    --exclude='*.mkv'
    --exclude='*.avi'

    --exclude='*.zip'
    --exclude='*.tar'
    --exclude='*.tar.gz'

    --exclude='.env'
    --exclude='*.pem'
    --exclude='*.key'
    --exclude='*.p12'
)

(
    cd "$workspace_root"
    tar "${exclude_args[@]}" -cf - .
) | tar -C "${staging_dir}/project" -xf -

echo "[2/5] 生成文件清单..."

(
    cd "$workspace_root"

    find . -type f \
        ! -path './build/*' \
        ! -path './install/*' \
        ! -path './log/*' \
        ! -path '*/.git/*' \
        ! -path '*/__pycache__/*' \
        ! -path '*/.pytest_cache/*' \
        ! -name '*.pyc' \
        ! -name '*.pyo' \
        ! -name '*.bag' \
        ! -name '*.db3' \
        ! -name '*.mcap' \
        ! -name '*.mp4' \
        ! -name '*.mkv' \
        ! -name '*.avi' \
        -printf '%p\t%s bytes\n' \
        | sort
) > "${staging_dir}/project/CODEX_FILE_MANIFEST.txt"

echo "[3/5] 记录 ROS 包和项目结构..."

{
    echo "Workspace: ${workspace_root}"
    echo "Collected at: $(date --iso-8601=seconds)"
    echo

    echo "ROS package.xml files:"
    find "${workspace_root}/src" \
        -name package.xml \
        -print 2>/dev/null || true

    echo
    echo "Launch files:"
    find "${workspace_root}/src" \
        \( \
            -name '*.launch.py' \
            -o -name '*.launch.xml' \
            -o -name '*.launch' \
        \) \
        -print 2>/dev/null || true

    echo
    echo "Build system files:"
    find "${workspace_root}/src" \
        \( \
            -name CMakeLists.txt \
            -o -name setup.py \
            -o -name setup.cfg \
            -o -name pyproject.toml \
        \) \
        -print 2>/dev/null || true

    echo
    echo "Important source files:"
    find "${workspace_root}/src" \
        \( \
            -name '*.py' \
            -o -name '*.cpp' \
            -o -name '*.cc' \
            -o -name '*.c' \
            -o -name '*.hpp' \
            -o -name '*.h' \
        \) \
        -print 2>/dev/null || true

} > "${staging_dir}/project/CODEX_PROJECT_STRUCTURE.txt"

echo "[4/5] 记录 Git 状态..."

if command -v git >/dev/null 2>&1 \
    && git -C "$workspace_root" rev-parse --is-inside-work-tree >/dev/null 2>&1; then

    {
        echo "Git status:"
        git -C "$workspace_root" status --short || true

        echo
        echo "Latest commit:"

        if git -C "$workspace_root" rev-parse --verify HEAD >/dev/null 2>&1; then
            git -C "$workspace_root" log -1 --oneline
        else
            echo "(当前 Git 仓库还没有任何 commit)"
        fi
    } > "${staging_dir}/project/CODEX_GIT_STATUS.txt"

else
    echo "当前工作区不是 Git 仓库，或者系统没有安装 Git。" \
        > "${staging_dir}/project/CODEX_GIT_STATUS.txt"
fi

echo "[5/5] 创建压缩包..."

tar -C "$staging_dir" -czf "$archive_path" project

echo
echo "打包完成："
echo "$archive_path"

echo
echo "文件大小："
du -h "$archive_path"

echo
echo "SHA256 校验值："
if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$archive_path" \
        | tee "${archive_path}.sha256"
else
    echo "系统没有 sha256sum，跳过校验值生成。"
fi
