# 1080P、2.9 mm 无畸变近似内参；正式使用前建议重新标定。
camera = {
    "device": 1,
    "width": 1920,
    "height": 1080,
    "fourcc": "MJPG",
    "fps": 30.0,
    # DirectShow 启动时前几帧可能全黑或曝光不稳定。
    "warmup_frames": 5,
    # DirectShow 手动曝光可避免取色后画面亮度继续漂移；现场可微调。
    "auto_exposure": False,
    "exposure": -4.0,
    "camera_matrix": [
        [966.66666667, 0.0, 960.0],
        [0.0, 966.66666667, 540.0],
        [0.0, 0.0, 1.0],
    ],
    "dist_coeffs": [0.0, 0.0, 0.0, 0.0, 0.0],
}

# 标定板参数。坐标单位为毫米，X 向右、Y 向上，Marker 平面 Z=0。
# 每组角点顺序对应 OpenCV ArUco：左上、右上、右下、左下。
# Marker 布局：ID 0 左上、ID 1 右上、ID 2 右下、ID 3 左下。
board = {
    "dictionary": "DICT_4X4_50",
    # 下方 Marker 坐标沿用现场板面定义：X 向右，Y 向前。
    # 机械臂 base_link 使用 FLU：X 向前，Y 向左，因此：
    # [X_base, Y_base] = [Y_board, -X_board]。
    "board_to_base_xy": [
        [0.0, 1.0],
        [-1.0, 0.0],
    ],
    "planar_fit_mode": "marker_centers",
    "stable_samples_per_marker": 6,
    "max_capture_frames": 200,
    "max_corner_jitter_px": 1.5,
    "capture_reset_motion_px": 4.0,
    "max_planar_reprojection_error_px": 2.0,
    "max_reprojection_error_px": 8.0,
    "origin_base_mm": [0.0, 0.0, 0.0],
    "marker_corners_base_mm": {
        0: [
            [-210.0, 239.5, 0.0],
            [-160.0, 239.5, 0.0],
            [-160.0, 189.5, 0.0],
            [-210.0, 189.5, 0.0],
        ],
        1: [
            [160.0, 239.5, 0.0],
            [210.0, 239.5, 0.0],
            [210.0, 189.5, 0.0],
            [160.0, 189.5, 0.0],
        ],
        2: [
            [160.0, -7.5, 0.0],
            [210.0, -7.5, 0.0],
            [210.0, -57.5, 0.0],
            [160.0, -57.5, 0.0],
        ],
        3: [
            [-210.0, -7.5, 0.0],
            [-160.0, -7.5, 0.0],
            [-160.0, -57.5, 0.0],
            [-210.0, -57.5, 0.0],
        ],
    },
}

# 抓取任务参数。高度均为 base_link 下的绝对 Z 坐标，单位为毫米。
task = {
    "safe_z_mm": 50.0,
    "approach_z_mm": 0.0,
    "grasp_z_mm": -85.0,
    # 第一次 IK 收到 ACK 后、继续下降前的等待时间。
    "approach_pause_s": 1.0,
    # 必填：木块放置点在 base_link 下的 X/Y 坐标。
    "place_x_mm": 196.396,
    "place_y_mm": -65.808,
}

