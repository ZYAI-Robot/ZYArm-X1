#!/bin/bash

# ZYArm examples build helper.
# Builds CMake-based Orbbec examples under software/examples.

set -e

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
cd "$SCRIPT_DIR"

echo "=============================="
echo "ZYArm examples build helper"
echo "=============================="

BUILD_ALL=true
EXAMPLE_NAME=""
CLEAN_BUILD=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --example|-e|--scenario|-s)
            EXAMPLE_NAME="$2"
            BUILD_ALL=false
            shift 2
            ;;
        --clean|-c)
            CLEAN_BUILD=true
            shift
            ;;
        --help|-h)
            echo "用法: $0 [选项]"
            echo "选项:"
            echo "  -e, --example NAME    构建指定示例，例如 robot_arm_pick 或 camera_pick/orbbec"
            echo "  -c, --clean           清理构建目录"
            echo "  -h, --help            显示帮助信息"
            echo ""
            echo "Deptrum 示例使用独立脚本: camera_pick/deptrum/build.sh <camera_sdk_path>"
            exit 0
            ;;
        *)
            echo "未知选项: $1"
            exit 1
            ;;
    esac
done

if [ "$CLEAN_BUILD" = true ]; then
    echo "清理构建目录..."
    rm -rf build
fi

mkdir -p build/bin
mkdir -p third_party

if [ ! -d "third_party/OrbbecSDK" ]; then
    echo "正在克隆 Orbbec SDK..."
    git clone https://gitee.com/orbbecdeveloper/OrbbecSDK.git third_party/OrbbecSDK
else
    echo "Orbbec SDK 已存在，跳过克隆。"
fi

echo "构建 Orbbec SDK..."
mkdir -p third_party/OrbbecSDK/build
cmake -S third_party/OrbbecSDK -B third_party/OrbbecSDK/build -DCMAKE_BUILD_TYPE=Release
cmake --build third_party/OrbbecSDK/build -j"$(nproc)"

if [ "$BUILD_ALL" = true ]; then
    EXAMPLES="robot_arm_pick camera_pick/orbbec"
else
    if [ ! -f "$EXAMPLE_NAME/CMakeLists.txt" ]; then
        echo "错误: 示例 '$EXAMPLE_NAME' 不存在或没有 CMakeLists.txt"
        exit 1
    fi
    EXAMPLES="$EXAMPLE_NAME"
fi

for example in $EXAMPLES; do
    if [ ! -f "$example/CMakeLists.txt" ]; then
        echo "警告: 示例 '$example' 没有 CMakeLists.txt，跳过。"
        continue
    fi

    echo "构建示例: $example"
    safe_name=${example//\//_}
    example_build_dir="build/examples/$safe_name"
    mkdir -p "$example_build_dir"

    cmake -S "$example" -B "$example_build_dir" \
        -DCMAKE_BUILD_TYPE=Release \
        -DOrbbecSDK_DIR="$SCRIPT_DIR/third_party/OrbbecSDK/build"

    cmake --build "$example_build_dir" -j"$(nproc)"

    executable=$(find "$example_build_dir/bin" -maxdepth 1 -type f -executable 2>/dev/null | head -n 1)
    if [ -n "$executable" ]; then
        output_name="build/bin/${safe_name}_$(basename "$executable")"
        cp "$executable" "$output_name"
        echo "  可执行文件已复制到: $output_name"
    fi
done

echo "=============================="
echo "构建完成，可执行文件位于: build/bin/"
echo "=============================="
