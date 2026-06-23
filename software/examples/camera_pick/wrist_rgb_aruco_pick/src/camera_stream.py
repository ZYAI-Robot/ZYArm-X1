from __future__ import annotations

import math
import sys
from typing import Any, Mapping, Optional


class CameraStream:
    DEFAULT_WARMUP_FRAMES = 1

    def __init__(
        self,
        camera_config: Mapping[str, Any],
        *,
        cv2_module: Any,
        capture_factory: Any = None,
    ) -> None:
        self.camera_config = dict(camera_config)
        self.cv2 = cv2_module
        self._capture_factory = capture_factory or cv2_module.VideoCapture
        self._capture: Any = None
        self._pending_frame: Any = None

    def open(self) -> None:
        if self._capture is not None:
            return

        device = self.camera_config.get("index", self.camera_config.get("device", 0))
        width = self.camera_config.get("width")
        height = self.camera_config.get("height")
        fps = self.camera_config.get("fps")
        fourcc = self.camera_config.get("fourcc")
        warmup_frames = int(self.camera_config.get("warmup_frames", self.DEFAULT_WARMUP_FRAMES))
        fps_value = float(fps) if fps is not None else None
        self._validate_stream_config(warmup_frames, fps_value, fourcc)

        capture = self._open_capture(device, width, height, fps_value, fourcc)
        if capture is None or not capture.isOpened():
            raise RuntimeError(f"Failed to open camera: {device!r}")

        try:
            self._configure_capture(capture, width, height, fps_value, fourcc)
            self._pending_frame = self._read_warmup_frame(capture, warmup_frames)
        except BaseException:
            capture.release()
            raise

        self._capture = capture

    def read_frame(self) -> Any:
        if self._capture is None:
            self.open()
        if self._pending_frame is not None:
            frame = self._pending_frame
            self._pending_frame = None
            return frame
        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to read frame from camera")
        return frame

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
        self._capture = None
        self._pending_frame = None

    def _open_capture(
        self,
        device: Any,
        width: Any,
        height: Any,
        fps: Optional[float],
        fourcc: Optional[str],
    ) -> Any:
        if sys.platform != "win32":
            return self._capture_factory(device)

        stream_params: list[int] = []
        if fourcc is not None:
            stream_params.extend(
                [self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc(*fourcc)]
            )
        if width is not None:
            stream_params.extend([self.cv2.CAP_PROP_FRAME_WIDTH, int(width)])
        if height is not None:
            stream_params.extend([self.cv2.CAP_PROP_FRAME_HEIGHT, int(height)])
        if fps is not None:
            stream_params.extend([self.cv2.CAP_PROP_FPS, round(fps)])
        return self._capture_factory(device, self.cv2.CAP_DSHOW, stream_params)

    def _configure_capture(
        self,
        capture: Any,
        width: Any,
        height: Any,
        fps: Optional[float],
        fourcc: Optional[str],
    ) -> None:
        if sys.platform != "win32":
            if fourcc is not None:
                capture.set(self.cv2.CAP_PROP_FOURCC, self.cv2.VideoWriter_fourcc(*fourcc))
            if width is not None:
                capture.set(self.cv2.CAP_PROP_FRAME_WIDTH, int(width))
            if height is not None:
                capture.set(self.cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            if fps is not None:
                capture.set(self.cv2.CAP_PROP_FPS, fps)

        auto_exposure = self.camera_config.get("auto_exposure")
        if auto_exposure is not None:
            capture.set(self.cv2.CAP_PROP_AUTO_EXPOSURE, 0.75 if bool(auto_exposure) else 0.25)
        exposure = self.camera_config.get("exposure")
        if exposure is not None:
            capture.set(self.cv2.CAP_PROP_EXPOSURE, float(exposure))
        buffersize = self.camera_config.get("buffersize")
        if buffersize is not None and hasattr(self.cv2, "CAP_PROP_BUFFERSIZE"):
            capture.set(self.cv2.CAP_PROP_BUFFERSIZE, int(buffersize))

    @staticmethod
    def _validate_stream_config(
        warmup_frames: int,
        fps: Optional[float],
        fourcc: Optional[str],
    ) -> None:
        if warmup_frames <= 0:
            raise ValueError("camera.warmup_frames must be a positive integer")
        if fps is not None and (not math.isfinite(fps) or fps <= 0.0):
            raise ValueError("camera.fps must be a positive finite number")
        if fourcc is not None and (
            not isinstance(fourcc, str)
            or len(fourcc) != 4
            or not fourcc.isascii()
        ):
            raise ValueError("camera.fourcc must be a four-character ASCII string")

    @staticmethod
    def _read_warmup_frame(capture: Any, warmup_frames: int) -> Any:
        frame = None
        for _index in range(warmup_frames):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("Camera opened but did not return a frame")
        return frame
