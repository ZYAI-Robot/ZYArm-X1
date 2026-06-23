# ZYArm Wrist RGB ArUco Pick

该实验使用安装在末端夹爪附近的普通 RGB 相机，识别贴在小方块顶部的固定尺寸 ArUco，并从固定观察姿态计算抓取偏移。

```text
move_ik(observe_pose)
  -> 腕部相机读取画面
  -> 检测目标 ArUco
  -> solvePnP 得到 marker 在 camera 坐标系下的位置
  -> 按 OpenCV -> 摄像头 -> 夹爪 -> base_link 的坐标链路换算 XY 偏移
  -> 抓取方块
  -> 移动到 place_pose 放置
  -> 回到 observe_pose 等待下一轮
```

第一版只支持固定 `observe_pose` 下的一次偏移计算，不做任意初始姿态在线手眼标定，也不做目标上方二次 ArUco 微调。二次微调可以在确认第一版抓放稳定后再作为增强加入。

## 安装

在仓库根目录执行：

```bash
python -m pip install -r software/examples/camera_pick/wrist_rgb_aruco_pick/requirements.txt
python -m pip install -e software/zyarm_sdk/python
```

## 配置

默认配置位于：

```text
software/examples/camera_pick/wrist_rgb_aruco_pick/config/wrist_rgb_aruco_pick.yaml
```

关键字段：

- `camera`：相机编号、分辨率、`fourcc`、帧率、预热帧数、内参矩阵和畸变参数；Windows 下会优先用 DirectShow 一次性协商 `MJPG`、分辨率和 FPS，减少启动等待。
- `marker`：ArUco 字典、目标 ID、实际边长和重投影误差阈值。
- `observe_pose`：每轮识别开始前机械臂固定进入的观察姿态。
- `place_pose`：抓取成功后释放方块的固定放置姿态。
- `mapping.camera_to_tool`：摄像头机械坐标系到末端夹爪坐标系的固定外参，包含平移和安装旋转。
- `mapping.opencv_to_camera`：OpenCV 相机坐标系到摄像头机械坐标系的轴向转换。
- `mapping.reference_camera_xy_mm`：没有配置 `camera_to_tool` 时，marker 在相机坐标下处于该 XY 即认为夹爪正对目标。
- `mapping.camera_xy_to_base_xy`：没有配置 `camera_to_tool` 时，把 camera XY 偏移转换为 `base_link` XY 偏移的 2x2 矩阵。
- `mapping.grasp_offset_base_xy_mm`：夹爪中心、相机中心和 marker 中心之间的补偿偏移。
- `mapping.max_xy_offset_mm`：允许的最大平面偏移，超过后不动作。
- `task`：安全高度、抓取高度和等待时间。
- `loop.max_cycles`：最大抓放轮数；`null` 表示不限制，会在画面中持续等待 SPACE 触发下一次抓取。
- `loop.wait_for_user`：每轮结束后是否额外等待终端确认；默认关闭，主要依赖预览窗口的 SPACE / `q` / `Esc` 控制。

## 纯视觉检测

单张图片：

```bash
python software/examples/camera_pick/wrist_rgb_aruco_pick/detect_aruco_block.py --image path/to/image.jpg --save output/aruco_overlay.png
```

摄像头：

```bash
python software/examples/camera_pick/wrist_rgb_aruco_pick/detect_aruco_block.py --camera 0 --show
```

输出 JSON 包含 `corners_px`、`center_px`、`rvec`、`tvec_mm`、`distance_mm`、`reprojection_error_px` 和 `reason`。

## Dry-run 验证

先运行 dry-run，确认坐标方向和计划动作：

```bash
python software/examples/camera_pick/wrist_rgb_aruco_pick/pick_aruco_block.py --dry-run
```

dry-run 只打开相机并打印：

- `delta_camera_xy`
- `delta_base_xy`
- 计划执行的 `observe_pose`
- 目标上方安全位姿
- `place_pose`

它不会连接机械臂串口，也不会调用任何夹爪或运动命令。

## 方向验证

正式移动机械臂前，建议把贴有 ArUco 的方块分别向画面左、右、上、下移动少量距离，观察 dry-run 输出：

```text
delta_camera_xy -> 相机坐标下的目标偏移
delta_base_xy   -> 转换后的 base_link 平面偏移
target safe IK  -> 计划移动到的目标上方位置
```

启用 3D 外参时，程序使用完整链路：

```text
p_B_M = p_B_T + R_B_T * (p_T_C + R_T_C * R_C_O * p_O_M)
```

其中 `p_O_M` 来自 OpenCV `solvePnP` 的 `tvec`，`R_C_O` 来自 `mapping.opencv_to_camera.rotation_matrix`，`p_T_C / R_T_C` 来自 `mapping.camera_to_tool`，`p_B_T / R_B_T` 来自 `observe_pose`。如果方块向右移动时机械臂计划向相反方向移动，优先检查 `mapping.opencv_to_camera.rotation_matrix` 和 `mapping.camera_to_tool.rotation_deg`。

## 真机抓放

确认 dry-run 输出正确后，在人工监护下运行：

```bash
python software/examples/camera_pick/wrist_rgb_aruco_pick/pick_aruco_block.py --port COM3
```

Linux 串口示例：

```bash
python software/examples/camera_pick/wrist_rgb_aruco_pick/pick_aruco_block.py --port /dev/ttyUSB0
```

每轮流程：

1. 移动到 `observe_pose`。
2. 识别当前 ArUco 并计算偏移。
3. 打开夹爪，以 `(rx, ry, rz) = (0, 0, 0)` 移动到目标上方安全高度，使夹爪默认朝下。
4. 下降到抓取高度。
5. 闭合夹爪并抬升。
6. 移动到 `place_pose` 并松开夹爪。
7. 回到 `observe_pose`。
8. 持续显示画面并等待下一次 SPACE；按 `q` 或 `Esc` 时先执行 `reset()` 复位，再关闭程序。

## 安全

- 首次真机调试必须人工监护。
- ArUco 检测失败、ID 不匹配、PnP 误差过高、偏移超限或配置非法时不会执行抓取。
- 任一 SDK 命令未被接受时，控制器停止后续动作并报告阶段名。
- 按 `q` 或 `Esc` 正常退出时会执行 `reset()` 复位；`Ctrl+C` 或关闭终端不是硬实时急停，异常时请使用机械臂硬件安全方式。
