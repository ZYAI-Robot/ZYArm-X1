from __future__ import annotations

import math
import sys
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Mapping, Optional, Sequence

import cv2
import numpy as np


Point2D = tuple[float, float]


class BlockDetectionCancelled(RuntimeError):
    """Raised when the user cancels live block detection."""


class BlockDetectionTimeout(RuntimeError):
    """Raised when no stable block is found in the capture window."""


@dataclass(frozen=True)
class BlockDetection:
    detected: bool
    stable: bool = False
    graspable: bool = False
    center_px: Optional[Point2D] = None
    box_points_px: tuple[Point2D, ...] = ()
    axis_points_px: Optional[tuple[Point2D, Point2D]] = None
    angle_deg: Optional[float] = None
    area_px: float = 0.0
    rectangularity: float = 0.0
    timestamp: float = 0.0
    reject_reason: Optional[str] = None


@dataclass(frozen=True)
class HSVColorModel:
    hue_ranges: tuple[tuple[int, int], ...]
    saturation_range: tuple[int, int]
    value_range: tuple[int, int]


class ColorBlockVision:
    """RGB camera and color-block observations for the pick controller."""

    DEFAULT_WARMUP_FRAMES = 5
    MIN_ROI_PIXELS = 100
    MIN_COLOR_SATURATION = 8
    MIN_CONTOUR_AREA_PX = 300.0
    MAX_FRAME_AREA_RATIO = 0.5
    MIN_RECTANGULARITY = 0.65
    MAX_ASPECT_RATIO = 3.0
    SQUARE_ASPECT_RATIO = 1.15
    EDGE_MARGIN_PX = 8
    MORPH_OPEN_SIZE = 3
    MORPH_CLOSE_SIZE = 7
    STABLE_FRAME_COUNT = 5
    MAX_CENTER_JITTER_PX = 4.0
    MAX_ANGLE_JITTER_DEG = 6.0

    def __init__(
        self,
        camera_config: Mapping[str, Any],
        *,
        capture_factory: Any = cv2.VideoCapture,
    ) -> None:
        self.camera_config = dict(camera_config)
        self._capture_factory = capture_factory
        self._capture: Any = None
        self._pending_frame: Optional[np.ndarray] = None
        self._color_model: Optional[HSVColorModel] = None
        self._history: deque[tuple[Point2D, float]] = deque(maxlen=self.STABLE_FRAME_COUNT)
        self._open_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (self.MORPH_OPEN_SIZE, self.MORPH_OPEN_SIZE),
        )
        self._close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (self.MORPH_CLOSE_SIZE, self.MORPH_CLOSE_SIZE),
        )

    @property
    def color_model(self) -> Optional[HSVColorModel]:
        return self._color_model

    def open(self) -> None:
        if self._capture is not None:
            return

        device = self.camera_config.get("device", 0)
        width = self.camera_config.get("width")
        height = self.camera_config.get("height")
        fps = self.camera_config.get("fps")
        fourcc = self.camera_config.get("fourcc")
        warmup_frames = int(
            self.camera_config.get("warmup_frames", self.DEFAULT_WARMUP_FRAMES)
        )
        fps_value = float(fps) if fps is not None else None
        self._validate_stream_config(warmup_frames, fps_value, fourcc)

        capture = self._open_capture(device, width, height, fps_value, fourcc)
        if capture is None or not capture.isOpened():
            raise RuntimeError(f"Failed to open RGB camera: {device!r}")

        try:
            self._configure_capture(capture, width, height, fps_value, fourcc)
            frame = self._read_warmup_frame(capture, warmup_frames)
            self._validate_capture_stream(capture, frame, width, height, fourcc)
        except BaseException:
            capture.release()
            raise

        self._capture = capture
        self._pending_frame = frame

    def read_frame(self) -> np.ndarray:
        if self._capture is None:
            raise RuntimeError("RGB camera is not open")
        if self._pending_frame is not None:
            frame = self._pending_frame
            self._pending_frame = None
            return frame

        ok, frame = self._capture.read()
        if not ok or frame is None:
            raise RuntimeError("Failed to read an RGB camera frame")
        return frame

    def close(self) -> None:
        if self._capture is not None:
            self._capture.release()
        self._capture = None
        self._pending_frame = None
        self._history.clear()

    def select_color_roi(
        self,
        frame_bgr: np.ndarray,
        *,
        roi: Optional[Sequence[int]] = None,
        window_name: str = "Select target color block",
    ) -> HSVColorModel:
        if roi is None:
            selected = cv2.selectROI(window_name, frame_bgr, showCrosshair=True, fromCenter=False)
            cv2.destroyWindow(window_name)
            roi = selected
        return self.build_color_model(frame_bgr, roi)

    def build_color_model(
        self,
        frame_bgr: np.ndarray,
        roi: Sequence[int],
    ) -> HSVColorModel:
        self._validate_bgr_frame(frame_bgr)
        patch = self._extract_color_patch(frame_bgr, roi)
        samples = self._select_color_samples(patch)

        hue_ranges = self._estimate_hue_ranges(samples[:, 0].astype(np.float64))
        saturation_range = (
            max(0, int(math.floor(float(np.percentile(samples[:, 1], 5)))) - 20),
            min(255, int(math.ceil(float(np.percentile(samples[:, 1], 99)))) + 20),
        )
        value_range = (
            max(0, int(math.floor(float(np.percentile(samples[:, 2], 5)))) - 25),
            min(255, int(math.ceil(float(np.percentile(samples[:, 2], 99)))) + 25),
        )

        model = HSVColorModel(
            hue_ranges=hue_ranges,
            saturation_range=saturation_range,
            value_range=value_range,
        )
        self._color_model = model
        self._history.clear()
        return model

    def detect(self, frame_bgr: np.ndarray) -> tuple[BlockDetection, np.ndarray]:
        if self._color_model is None:
            raise RuntimeError("Select a color ROI before detecting a block")
        self._validate_bgr_frame(frame_bgr)

        timestamp = time.monotonic()
        mask = self._build_mask(frame_bgr)
        contours, _hierarchy = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        candidates = self._find_candidates(contours, frame_bgr.shape[:2])
        if not candidates:
            self._history.clear()
            return (
                BlockDetection(
                    detected=False,
                    timestamp=timestamp,
                    reject_reason="no_valid_color_block",
                ),
                mask,
            )

        detection = self._build_detection(
            max(candidates, key=lambda item: item[0]),
            timestamp,
        )
        return detection, mask

    def _build_detection(
        self,
        candidate: tuple[float, float, Any, bool],
        timestamp: float,
    ) -> BlockDetection:
        area, rectangularity, rect, touches_edge = candidate
        center_raw, _size, _raw_angle = rect
        box = cv2.boxPoints(rect).astype(np.float64)
        center = (float(center_raw[0]), float(center_raw[1]))
        width, height = (float(value) for value in rect[1])
        aspect_ratio = max(width, height) / max(min(width, height), 1e-6)
        orientation_required = aspect_ratio >= self.SQUARE_ASPECT_RATIO
        axis_points, angle_deg = self._block_axis(
            box,
            center,
            max(width, height),
            orientation_required,
        )
        stable = self._update_stability(
            center,
            angle_deg,
            orientation_required=orientation_required,
        )

        reason: Optional[str] = None
        if touches_edge:
            reason = "target_too_close_to_image_edge"
        elif not stable:
            reason = "target_not_stable"

        return BlockDetection(
            detected=True,
            stable=stable,
            graspable=stable and not touches_edge,
            center_px=center,
            box_points_px=tuple((float(point[0]), float(point[1])) for point in box),
            axis_points_px=axis_points,
            angle_deg=angle_deg,
            area_px=float(area),
            rectangularity=float(rectangularity),
            timestamp=timestamp,
            reject_reason=reason,
        )

    def detect_stable(
        self,
        *,
        max_frames: int = 180,
        show: bool = True,
        window_name: str = "ZYArm block vision",
    ) -> BlockDetection:
        self._history.clear()
        for _index in range(max_frames):
            frame = self.read_frame()
            result, mask = self.detect(frame)
            if show:
                cv2.imshow(window_name, self.draw_result(frame, result))
                cv2.imshow(f"{window_name} mask", mask)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    raise BlockDetectionCancelled(
                        "Block detection cancelled by user (q/Esc); "
                        "no arm commands were sent"
                    )
            if result.graspable:
                return result
        raise BlockDetectionTimeout("No stable, graspable color block was found")

    def draw_result(
        self,
        frame_bgr: np.ndarray,
        result: BlockDetection,
    ) -> np.ndarray:
        overlay = frame_bgr.copy()
        color = (0, 200, 0) if result.graspable else (0, 165, 255)

        if result.box_points_px:
            box = np.rint(np.asarray(result.box_points_px)).astype(np.int32)
            cv2.polylines(overlay, [box], True, color, 2)
        if result.center_px is not None:
            center = tuple(int(round(value)) for value in result.center_px)
            cv2.circle(overlay, center, 5, color, -1)
        if result.axis_points_px is not None:
            start = tuple(int(round(value)) for value in result.axis_points_px[0])
            end = tuple(int(round(value)) for value in result.axis_points_px[1])
            cv2.line(overlay, start, end, (255, 0, 0), 2)

        status = "graspable" if result.graspable else (result.reject_reason or "not_graspable")
        text = f"{status} area={result.area_px:.0f}"
        if result.angle_deg is not None:
            text += f" angle={result.angle_deg:.1f}"
        cv2.putText(
            overlay,
            text,
            (12, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
        return overlay

    def _build_mask(self, frame_bgr: np.ndarray) -> np.ndarray:
        assert self._color_model is not None
        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        saturation_min, saturation_max = self._color_model.saturation_range
        value_min, value_max = self._color_model.value_range
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for hue_min, hue_max in self._color_model.hue_ranges:
            current = cv2.inRange(
                hsv,
                np.array([hue_min, saturation_min, value_min], dtype=np.uint8),
                np.array([hue_max, saturation_max, value_max], dtype=np.uint8),
            )
            mask = cv2.bitwise_or(mask, current)

        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self._open_kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._close_kernel)

    def _find_candidates(
        self,
        contours: Sequence[np.ndarray],
        image_shape: tuple[int, int],
    ) -> list[tuple[float, float, Any, bool]]:
        image_height, image_width = image_shape
        max_area = float(image_height * image_width) * self.MAX_FRAME_AREA_RATIO
        candidates: list[tuple[float, float, Any, bool]] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.MIN_CONTOUR_AREA_PX or area > max_area:
                continue
            rect = cv2.minAreaRect(contour)
            width, height = (float(value) for value in rect[1])
            if width <= 0.0 or height <= 0.0:
                continue
            rectangularity = area / (width * height)
            aspect_ratio = max(width, height) / max(min(width, height), 1e-6)
            if rectangularity < self.MIN_RECTANGULARITY:
                continue
            if aspect_ratio > self.MAX_ASPECT_RATIO:
                continue

            x, y, bounding_width, bounding_height = cv2.boundingRect(contour)
            touches_edge = (
                x <= self.EDGE_MARGIN_PX
                or y <= self.EDGE_MARGIN_PX
                or x + bounding_width >= image_width - self.EDGE_MARGIN_PX
                or y + bounding_height >= image_height - self.EDGE_MARGIN_PX
            )
            candidates.append((area, rectangularity, rect, touches_edge))
        return candidates

    def _update_stability(
        self,
        center: Point2D,
        angle_deg: float,
        *,
        orientation_required: bool,
    ) -> bool:
        self._history.append((center, angle_deg))
        if len(self._history) < self.STABLE_FRAME_COUNT:
            return False

        centers = np.asarray([item[0] for item in self._history], dtype=np.float64)
        center_mean = np.mean(centers, axis=0)
        center_jitter = float(np.max(np.linalg.norm(centers - center_mean, axis=1)))

        angles = np.asarray([item[1] for item in self._history], dtype=np.float64)
        doubled = np.deg2rad(angles * 2.0)
        mean_angle = 0.5 * math.degrees(
            math.atan2(float(np.mean(np.sin(doubled))), float(np.mean(np.cos(doubled))))
        )
        if mean_angle < 0.0:
            mean_angle += 180.0
        angle_jitter = max(self._axis_angle_distance(value, mean_angle) for value in angles)
        center_is_stable = center_jitter <= self.MAX_CENTER_JITTER_PX
        if not orientation_required:
            return center_is_stable
        return center_is_stable and angle_jitter <= self.MAX_ANGLE_JITTER_DEG

    @staticmethod
    def _validate_bgr_frame(frame_bgr: np.ndarray) -> None:
        if frame_bgr is None or frame_bgr.ndim != 3 or frame_bgr.shape[2] != 3:
            raise ValueError("frame_bgr must be a BGR image")

    def _extract_color_patch(
        self,
        frame_bgr: np.ndarray,
        roi: Sequence[int],
    ) -> np.ndarray:
        if len(roi) != 4:
            raise ValueError("roi must contain x, y, width, height")

        x, y, width, height = (int(value) for value in roi)
        frame_height, frame_width = frame_bgr.shape[:2]
        if width <= 0 or height <= 0:
            raise ValueError("Color ROI was cancelled or is empty")
        if x < 0 or y < 0 or x + width > frame_width or y + height > frame_height:
            raise ValueError("Color ROI is outside the image")

        patch = frame_bgr[y : y + height, x : x + width]
        if patch.size // 3 < self.MIN_ROI_PIXELS:
            raise ValueError("Color ROI is too small")

        # Ignore the ROI border so nearby background does not dominate the model.
        trim_x = max(0, int(round(width * 0.1)))
        trim_y = max(0, int(round(height * 0.1)))
        if width - 2 * trim_x >= 3 and height - 2 * trim_y >= 3:
            patch = patch[trim_y : height - trim_y, trim_x : width - trim_x]
        return patch

    def _select_color_samples(self, patch: np.ndarray) -> np.ndarray:
        hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        saturation = hsv[:, 1].astype(np.float64)
        value = hsv[:, 2].astype(np.float64)
        saturation_floor = max(
            self.MIN_COLOR_SATURATION,
            int(math.floor(float(np.percentile(saturation, 20)))),
        )
        samples = hsv[(saturation >= saturation_floor) & (value >= 20)]
        if len(samples) < self.MIN_ROI_PIXELS:
            raise ValueError("Selected ROI does not contain enough usable color pixels")
        if float(np.median(samples[:, 1])) < self.MIN_COLOR_SATURATION:
            raise ValueError("Selected ROI has too little color saturation")
        return samples

    @classmethod
    def _block_axis(
        cls,
        box: np.ndarray,
        center: Point2D,
        axis_length: float,
        orientation_required: bool,
    ) -> tuple[tuple[Point2D, Point2D], float]:
        if orientation_required:
            return cls._long_axis(box, center)

        # Squares have no unique long edge, so use a stable horizontal reference.
        half_length = axis_length * 0.5
        return (
            (
                (center[0] - half_length, center[1]),
                (center[0] + half_length, center[1]),
            ),
            0.0,
        )

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
                [cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc)]
            )
        if width is not None:
            stream_params.extend([cv2.CAP_PROP_FRAME_WIDTH, int(width)])
        if height is not None:
            stream_params.extend([cv2.CAP_PROP_FRAME_HEIGHT, int(height)])
        if fps is not None:
            stream_params.extend([cv2.CAP_PROP_FPS, round(fps)])
        return self._capture_factory(device, cv2.CAP_DSHOW, stream_params)

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
                capture.set(
                    cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*fourcc),
                )
            if width is not None:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            if height is not None:
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
            if fps is not None:
                capture.set(cv2.CAP_PROP_FPS, fps)

        auto_exposure = self.camera_config.get("auto_exposure")
        if auto_exposure is not None:
            capture.set(
                cv2.CAP_PROP_AUTO_EXPOSURE,
                0.75 if bool(auto_exposure) else 0.25,
            )
        exposure = self.camera_config.get("exposure")
        if exposure is not None:
            capture.set(cv2.CAP_PROP_EXPOSURE, float(exposure))

    @staticmethod
    def _read_warmup_frame(capture: Any, warmup_frames: int) -> np.ndarray:
        frame = None
        for _index in range(warmup_frames):
            ok, frame = capture.read()
            if not ok or frame is None:
                raise RuntimeError("RGB camera opened but did not return a frame")
        assert frame is not None
        return frame

    def _validate_capture_stream(
        self,
        capture: Any,
        frame: np.ndarray,
        width: Any,
        height: Any,
        fourcc: Optional[str],
    ) -> None:
        actual_height, actual_width = frame.shape[:2]
        if width is not None and actual_width != int(width):
            raise RuntimeError(
                f"RGB camera width mismatch: configured={int(width)}, actual={actual_width}"
            )
        if height is not None and actual_height != int(height):
            raise RuntimeError(
                f"RGB camera height mismatch: configured={int(height)}, actual={actual_height}"
            )
        if fourcc is None:
            return

        actual_fourcc = self._decode_fourcc(
            int(capture.get(cv2.CAP_PROP_FOURCC))
        )
        if actual_fourcc != fourcc:
            raise RuntimeError(
                "RGB camera pixel format mismatch: "
                f"configured={fourcc}, actual={actual_fourcc or 'unknown'}"
            )

    @staticmethod
    def _estimate_hue_ranges(hues: np.ndarray) -> tuple[tuple[int, int], ...]:
        radians = hues * (2.0 * math.pi / 180.0)
        center = math.atan2(float(np.mean(np.sin(radians))), float(np.mean(np.cos(radians))))
        center_hue = (center % (2.0 * math.pi)) * 180.0 / (2.0 * math.pi)
        differences = ((hues - center_hue + 90.0) % 180.0) - 90.0
        half_width = float(np.percentile(np.abs(differences), 95)) + 4.0
        half_width = min(35.0, max(4.0, half_width))

        lower = center_hue - half_width
        upper = center_hue + half_width
        if lower < 0.0:
            return (
                (0, min(179, int(math.ceil(upper)))),
                (max(0, int(math.floor(lower + 180.0))), 179),
            )
        if upper > 179.0:
            return (
                (0, min(179, int(math.ceil(upper - 180.0)))),
                (max(0, int(math.floor(lower))), 179),
            )
        return ((int(math.floor(lower)), int(math.ceil(upper))),)

    @staticmethod
    def _long_axis(
        box_points: np.ndarray,
        center: Point2D,
    ) -> tuple[tuple[Point2D, Point2D], float]:
        edges = [
            box_points[(index + 1) % 4] - box_points[index]
            for index in range(4)
        ]
        vector = max(edges, key=lambda item: float(np.linalg.norm(item)))
        length = float(np.linalg.norm(vector))
        if length <= 1e-9:
            raise ValueError("Detected block has a degenerate rectangle")
        unit = vector / length
        center_array = np.asarray(center, dtype=np.float64)
        start = center_array - unit * (length * 0.5)
        end = center_array + unit * (length * 0.5)
        angle = math.degrees(math.atan2(float(unit[1]), float(unit[0]))) % 180.0
        if math.isclose(angle, 180.0, abs_tol=1e-9):
            angle = 0.0
        return (
            (
                (float(start[0]), float(start[1])),
                (float(end[0]), float(end[1])),
            ),
            float(angle),
        )

    @staticmethod
    def _axis_angle_distance(first: float, second: float) -> float:
        difference = abs(float(first) - float(second)) % 180.0
        return min(difference, 180.0 - difference)

    @staticmethod
    def _decode_fourcc(value: int) -> str:
        return "".join(
            chr((int(value) >> (8 * index)) & 0xFF)
            for index in range(4)
        ).rstrip("\x00")
