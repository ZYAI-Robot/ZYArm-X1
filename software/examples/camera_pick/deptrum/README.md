# Deptrum Camera SDK 编译指南

## SDK下载地址
https://drive.weixin.qq.com/s?k=ANUA-wdkABAg65LQ5a

## 前置准备

### 1. 下载SDK
根据目标Linux架构选择对应的SDK包：
- **x86_64架构**: `deptrum-stream-aurora900-linux-x86_64-v1.1.22-18.04.tar.gz`
- **aarch64架构**: `deptrum-stream-aurora900-linux-aarch64-v1.1.22.tar.gz`

### 2. 解压SDK
```bash
tar -xzf deptrum-stream-aurora900-linux-<架构>-v1.1.22.tar.gz
```
### 3. SDK环境配置（运行build.sh脚本前请先执行以下配置）

### USB设备权限配置
```bash
sudo chmod +x 99-deptrum-libusb.rules
sudo cp 99-deptrum-libusb.rules /etc/udev/rules.d/
sudo service udev reload
sudo service udev restart
```

### 安装系统依赖
```bash
sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libgl1-mesa-dev
sudo apt-get install libglfw3-dev
sudo apt-get install libglu1-mesa-dev
sudo apt-get install libxi-dev
sudo apt-get install libudev-dev
```
### 4. 安装依赖
在编译前，请确保系统已安装以下依赖：

#### OpenCV库（必需）
SDK的sample_touch示例需要OpenCV库支持。请确保已安装OpenCV开发库：

**Ubuntu系统：**
```bash
sudo apt-get update
sudo apt-get install libopencv-dev
```

**检查OpenCV是否安装：**
```bash
pkg-config --modversion opencv
```

#### 其他依赖
```bash
sudo apt-get install cmake build-essential
```

## 编译步骤

### 使用build.sh脚本编译

**基本用法：**
```bash
cd software/examples/camera_pick/deptrum
./build.sh < SDK绝对/相对路径 >
```

**完整示例：**
```bash
# 进入脚本目录
cd software/examples/camera_pick/deptrum

# 使用x86_64架构的SDK编译
./build.sh ../../../../deptrum-stream-aurora900-linux-x86_64-v1.1.22-18.04

# 或使用aarch64架构的SDK编译
./build.sh ../../../../deptrum-stream-aurora900-linux-aarch64-v1.1.22
```

### 脚本功能说明
1. 将`sample_touch`文件夹复制到SDK的`samples/src/`目录
2. 在SDK的`samples/src/CMakeLists.txt`中添加`add_subdirectory(sample_touch)`
3. 在SDK的`samples/`目录下执行cmake和make编译
4. 清理并创建新的`build`目录
5. 将编译好的可执行文件复制到`build/`目录
6. 将所需的动态库复制到`build/lib/`目录

## 注意事项

⚠️ **重要提示：**

1. **OpenCV依赖**：编译前必须确保系统已安装OpenCV开发库，否则编译会失败
2. **SDK路径**：请使用正确的SDK路径作为脚本参数
3. **架构匹配**：确保下载的SDK架构与目标运行环境一致
4. **权限问题**：如果脚本没有执行权限，请先执行：`chmod +x build.sh`
5. **备份文件**：脚本会自动备份SDK的CMakeLists.txt文件为CMakeLists.txt.bak

## 编译输出

编译成功后，输出文件位于：
- 可执行文件：`software/examples/camera_pick/deptrum/build/sample_touch`
- 动态库：`software/examples/camera_pick/deptrum/build/lib/`

## 运行示例

```bash
cd software/examples/camera_pick/deptrum/build
export LD_LIBRARY_PATH=./lib:$LD_LIBRARY_PATH
arm64 架构下：
    ./sample_touch /dev/tty341USB0 
arm x86_64 架构下：
    ./sample_touch /dev/ttyUSB0
```
请根据实际连接的相机设备号替换`<相机设备号>`，通常为0或1。
