#!/bin/bash

set -e
# 脚本所在目录
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
# 当前目录
CURRENT_DIR=$(pwd)

# 校验入参是否为目录
if [ $# -ne 1 ]; then
    echo "Usage: $0 <camera_sdk_path>"
    echo "Example: $0 /path/to/deptrum-stream-aurora900-linux-x86_64-v1.1.22-18.04"
    echo "Example: $0 /path/to/deptrum-stream-aurora900-linux-aarch64-v1.1.22"
    exit 1
fi

SDK_PATH=$(realpath "$1")

if [ ! -d "$SDK_PATH" ]; then
    echo "Error: SDK path does not exist: $SDK_PATH"
    exit 1
fi

# 校验SDK目录是否包含samples目录
if [ ! -d "$SDK_PATH/samples" ]; then
    echo "Error: samples directory not found in SDK path: $SDK_PATH/samples"
    exit 1
fi

# 校验SDK目录是否包含lib目录
if [ ! -d "$SDK_PATH/lib" ]; then
    echo "Error: lib directory not found in SDK path: $SDK_PATH/lib"
    exit 1
fi

TEST_CASE="sample_touch"

SDK_SAMPLES_SRC="$SDK_PATH/samples/src"
SDK_SAMPLES_CMAKELISTS="$SDK_PATH/samples/src/CMakeLists.txt"
SDK_LIB="$SDK_PATH/lib"
BUILD_DIR="$SCRIPT_DIR/build"
ZYARM_SDK_CPP="$SCRIPT_DIR/../../../zyarm_sdk/cpp"

# 第一步：复制sample_touch目录到SDK samples/src/ 目录下
echo "Step 1: Copying sample_touch directory to SDK samples/src..."
test_case_path="$SCRIPT_DIR/$TEST_CASE"
if [ -d "$test_case_path" ]; then
    echo "  Copying $TEST_CASE..."
    cp -rf "$test_case_path" "$SDK_SAMPLES_SRC/"
else
    echo "  Error: Test case directory not found: $test_case_path"
    exit 1
fi

# 第一步补充：复制 ZYArm C++ SDK
echo "Step 1.5: Copying ZYArm SDK files to SDK lib/zyarm_sdk..."
SDK_LIB_ZYARM="$SDK_LIB/zyarm_sdk"
if [ ! -d "$SDK_LIB_ZYARM" ]; then
    echo "  Creating zyarm_sdk directory in SDK lib..."
    mkdir -p "$SDK_LIB_ZYARM"
fi

if [ -d "$ZYARM_SDK_CPP/include" ]; then
    echo "  Copying zyarm_sdk include..."
    rm -rf "$SDK_LIB_ZYARM/include"
    cp -rf "$ZYARM_SDK_CPP/include" "$SDK_LIB_ZYARM/"
else
    echo "  Error: zyarm_sdk include not found: $ZYARM_SDK_CPP/include"
    exit 1
fi

if [ -d "$ZYARM_SDK_CPP/src" ]; then
    echo "  Copying zyarm_sdk src..."
    rm -rf "$SDK_LIB_ZYARM/src"
    cp -rf "$ZYARM_SDK_CPP/src" "$SDK_LIB_ZYARM/"
else
    echo "  Error: zyarm_sdk src not found: $ZYARM_SDK_CPP/src"
    exit 1
fi

# 第二步：在SDK samples/src/CMakeLists.txt中添加add_subdirectory(sample_touch)
echo "Step 2: Adding add_subdirectory entry to SDK samples/src/CMakeLists.txt..."
if [ -f "$SDK_SAMPLES_CMAKELISTS" ]; then
    cp "$SDK_SAMPLES_CMAKELISTS" "${SDK_SAMPLES_CMAKELISTS}.bak"
    echo "  Backed up original CMakeLists.txt to CMakeLists.txt.bak"
    
    if [ -d "$SDK_SAMPLES_SRC/$TEST_CASE" ]; then
        if ! grep -q "add_subdirectory($TEST_CASE)" "$SDK_SAMPLES_CMAKELISTS"; then
            echo "  Adding add_subdirectory($TEST_CASE)..."
            echo "add_subdirectory($TEST_CASE)" >> "$SDK_SAMPLES_CMAKELISTS"
        else
            echo "  add_subdirectory($TEST_CASE) already exists"
        fi
    fi
else
    echo "  Error: CMakeLists.txt not found at $SDK_SAMPLES_CMAKELISTS"
    exit 1
fi

# 第三步：在SDK samples/ 目录下编译
echo "Step 3: Building with cmake and make..."
cd "$SDK_PATH/samples"

if [ -d "build" ]; then
    echo "  Removing existing build directory in SDK samples..."
    rm -rf build
fi

mkdir -p build
cd build

echo "  Running cmake..."
cmake ..

echo "  Running make -j32..."
make -j32

cd "$CURRENT_DIR"

# 第四步：清理旧的build目录，创建新的build目录，复制可执行文件和动态库
echo "Step 4: Preparing build directory and copying files..."

if [ -d "$BUILD_DIR" ]; then
    echo "  Removing old build directory..."
    rm -rf "$BUILD_DIR"
fi

echo "  Creating new build directory..."
mkdir -p "$BUILD_DIR"
mkdir -p "$BUILD_DIR/lib"

# 复制可执行文件
executable_path="$SDK_PATH/samples/bin/$TEST_CASE"
if [ -f "$executable_path" ]; then
    echo "  Copying $TEST_CASE executable..."
    cp "$executable_path" "$BUILD_DIR/"
else
    echo "  Warning: Executable not found: $executable_path"
fi

# 复制动态库到lib目录，排除源码文件夹
echo "  Copying shared libraries to lib directory..."
for item in "$SDK_PATH/lib/"*; do
    if [ "$(basename "$item")" != "zyarm_sdk" ]; then
        cp -rf "$item" "$BUILD_DIR/lib/"
    fi
done

echo "Build completed successfully!"
echo "Executables are located in: $BUILD_DIR"
echo "Shared libraries are located in: $BUILD_DIR/lib"
