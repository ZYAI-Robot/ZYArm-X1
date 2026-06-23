# ZYArm Fixed RGB Color Pick

该工程使用固定在机械臂外部的普通 RGB 相机完成颜色方块抓取、放置和复位流程：

```text
启动框选木块取色
  -> 检测工作区 ArUco 标定板
  -> solvePnP / Homography 建立固定相机到工作平面的映射
  -> 识别同色木块中心和方向
  -> 投影到 base_link 的 z=0 平面
  -> 调用 zyarm_sdk 抓取、移动、放置并复位
```

这是固定外部相机的 eye-to-hand/PnP 平面映射示例，不是经典多姿态 `AX=XB` 标定流程。

源码由三个独立模块组成：

- `src/color_block_vision.py`：`ColorBlockVision`，负责 RGB 相机、启动取色和木块视觉观测。
- `src/fixed_camera_calibration.py`：`FixedCameraCalibrator`，负责 Marker/PnP 外参和像素到工作平面转换。
- `src/fixed_color_pick_controller.py`：`FixedColorPickController`，负责流程编排和 ZYArm SDK 控制。

运行前需要填写相机内参、畸变参数、采集分辨率、ArUco 字典、Marker 三维角点以及抓取任务高度。

## 安装

在仓库根目录执行：

```bash
python -m pip install -r software/examples/camera_pick/fixed_rgb_color_pick/requirements.txt
python -m pip install -e software/zyarm_sdk/python
```

## 配置约定

`config/fixed_rgb_color_pick.py` 只包含：

- `camera`：相机设备、分辨率、内参矩阵和畸变系数。Windows 默认使用
  DirectShow，并一次性协商 `MJPG`、`1920x1080` 和 `30 FPS`。
- `board`：ArUco 字典、标定板参考原点和 ID `0..3` 的三维角点。
- `task`：可现场调整的抓取参数。

`task` 包含：

- `safe_z_mm`：第一次接近及夹取后抬升的绝对 Z 坐标。
- `approach_z_mm`：第二次 IK 到达的过渡高度。
- `grasp_z_mm`：第三次 IK 到达的夹取高度。
- `approach_pause_s`：第一次 IK 收到 ACK 后的等待时间。
- `place_x_mm`、`place_y_mm`：木块放置点的 `base_link` 平面坐标。

Marker 空间布局：

```text
ID 0（左上） -------- ID 1（右上）
     |                       |
     |        工作区          |
     |                       |
ID 3（左下） -------- ID 2（右下）
```

每个 Marker 的四个三维角点必须与 OpenCV 检测角点顺序对应，并统一使用 `base_link` 下的毫米坐标。

## 运行

运行自动测试：

```bash
python software/examples/camera_pick/fixed_rgb_color_pick/tests/test_color_block_vision.py
python software/examples/camera_pick/fixed_rgb_color_pick/tests/test_fixed_camera_calibration.py
python software/examples/camera_pick/fixed_rgb_color_pick/tests/test_fixed_color_pick_controller.py
```

填写相机和标定板参数后，可分别打开实时调试窗口：

```bash
python software/examples/camera_pick/fixed_rgb_color_pick/tests/test_color_block_vision.py --camera
python software/examples/camera_pick/fixed_rgb_color_pick/tests/test_fixed_camera_calibration.py --camera
```

参数填写并完成分层验证后，再运行真机抓取：

```bash
python software/examples/camera_pick/fixed_rgb_color_pick/src/fixed_color_pick_controller.py --port COM3
```

程序启动后只进行一次颜色框选和固定相机标定。首次抓放自动执行，完成后保持运行：

- 手动把同一色块移动到新位置，待手离开工作区后按空格。
- 程序等待 `0.5` 秒，然后复用颜色模型和标定结果，重新识别并执行完整抓放。
- 某轮未识别到稳定色块时返回等待界面，不发送机械臂命令，也不退出程序。
- 按 `q` 或 `Esc` 退出程序。

机械臂执行抓放时，控制链运行在独立工作线程，主线程持续刷新相机画面。
运动期间按 `q` 或 `Esc` 会记录退出请求，当前动作安全完成后再退出。

Linux 串口示例：

```bash
python software/examples/camera_pick/fixed_rgb_color_pick/src/fixed_color_pick_controller.py --port /dev/ttyUSB0
```

## 坐标与边界

- 标定板平面和目标抓取平面约定为 `base_link` 的 `z=0`。
- 完整顺序为：接近抓取点、下降夹取、抬到 `approach_z_mm`、平移到放置点、
  下放到 `grasp_z_mm`、松开夹爪、抬回 `approach_z_mm`、执行 `reset()`。
- 三个高度必须满足 `safe_z_mm > approach_z_mm > grasp_z_mm`。
- 每次程序运行只执行一次 PnP，后续循环复用本次会话的标定结果。
- 当前流程执行接近、抓取、移动、放置、抬升和复位。
- 当前算法是固定相机的 eye-to-hand/PnP 平面映射，不是经典 `AX=XB` 标定流程。
- PnP 不补偿舵机回差、关节零位误差或机械结构变形。

## 安全

首次真机调试必须人工监护。先确认 Marker 回投、像素到 base 坐标和 `safe_z_mm`，再允许机械臂动作。任一识别、标定、坐标转换或 SDK 命令失败时，控制器都会停止后续动作。
