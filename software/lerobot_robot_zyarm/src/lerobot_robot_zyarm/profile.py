from __future__ import annotations

import threading
import time
from collections import defaultdict
from typing import Any, DefaultDict, Optional


# Code-level debug switch. Keep "off" for normal recording.
ZYARM_RECORD_PROFILER_MODE = "off"  # off | once | interval
ZYARM_TRAIN_PROFILER_MODE = "interval"  # off | once | interval
ZYARM_PROFILE_WARMUP_FRAMES = 50
ZYARM_PROFILE_SAMPLE_FRAMES = 5
ZYARM_PROFILE_INTERVAL_S = 5.0

_REQUIRED_METRICS = ("get_action_ms", "send_action_ms", "get_observation_ms")
_TRAIN_REQUIRED_METRICS = (
    "loop_ms",
    "obs_ms",
    "policy_ms",
    "send_ms",
    "state_age_at_obs_ms",
    "state_age_at_send_ms",
    "freshness_used_ms",
)
_TRAIN_REPORT_METRICS = (
    "loop_ms",
    "obs_ms",
    "policy_ms",
    "send_ms",
    "state_age_at_obs_ms",
    "state_age_at_send_ms",
    "freshness_used_ms",
    "overrun_ms",
)
_VALID_PROFILE_MODES = {"off", "once", "interval"}


class _DataQualityProfiler:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        self._printed_once = False
        self._counts: DefaultDict[str, int] = defaultdict(int)
        self._values: DefaultDict[str, list[float]] = defaultdict(list)
        self._camera_values: DefaultDict[str, list[float]] = defaultdict(list)
        self._last_get_action_start: Optional[float] = None
        self._latest_master_hz: Optional[float] = None
        self._latest_status_hz: Optional[float] = None
        self._latest_master_gaps: Optional[int] = None
        self._window_start: Optional[float] = None

    def enabled(self) -> bool:
        mode = _record_profile_mode()
        return mode == "interval" or (mode == "once" and not self._printed_once)

    def begin(self, metric: str) -> Optional[tuple[str, float, bool]]:
        mode = _record_profile_mode()
        if mode == "off" or (mode == "once" and self._printed_once):
            return None

        now = time.perf_counter()
        with self._lock:
            self._counts[metric] += 1
            should_sample = self._should_sample_locked(mode, metric)
            if should_sample and self._window_start is None:
                self._window_start = now
            if metric == "get_action":
                if should_sample and self._last_get_action_start is not None:
                    self._values["loop_ms"].append((now - self._last_get_action_start) * 1000.0)
                self._last_get_action_start = now
            return metric, now, should_sample

    def end(self, token: Optional[tuple[str, float, bool]], *, value_name: Optional[str] = None) -> bool:
        if token is None:
            return False
        metric, start, should_sample = token
        if not should_sample:
            return False
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        with self._lock:
            self._values[value_name or f"{metric}_ms"].append(elapsed_ms)
        return True

    def add_value(self, name: str, value: Any) -> None:
        if not self.enabled() or value is None:
            return
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            return
        with self._lock:
            self._values[name].append(parsed)

    def add_camera_value(self, name: str, value_ms: float) -> None:
        if not self.enabled():
            return
        with self._lock:
            self._camera_values[name].append(float(value_ms))

    def update_frame_stats(self, arm: Any) -> None:
        if not self.enabled():
            return
        transport = getattr(arm, "transport", None)
        get_stats = getattr(transport, "get_frame_stats", None)
        if not callable(get_stats):
            return
        try:
            stats = get_stats()
        except Exception:
            return
        with self._lock:
            self._latest_master_hz = getattr(stats, "master_data_rate_hz", None)
            self._latest_status_hz = getattr(stats, "status_rate_hz", None)
            self._latest_master_gaps = getattr(stats, "master_data_gap_count", None)

    def maybe_print(self) -> None:
        mode = _record_profile_mode()
        if mode == "off" or (mode == "once" and self._printed_once):
            return
        now = time.perf_counter()
        with self._lock:
            if mode == "once":
                self._print_once_if_ready_locked()
            elif mode == "interval":
                self._print_interval_if_ready_locked(now)

    def _should_sample_locked(self, mode: str, metric: str) -> bool:
        count = self._counts[metric]
        if count <= ZYARM_PROFILE_WARMUP_FRAMES:
            return False
        if mode == "once":
            return count <= ZYARM_PROFILE_WARMUP_FRAMES + ZYARM_PROFILE_SAMPLE_FRAMES
        return True

    def _print_once_if_ready_locked(self) -> None:
        if self._printed_once:
            return
        if any(len(self._values[name]) < ZYARM_PROFILE_SAMPLE_FRAMES for name in _REQUIRED_METRICS):
            return
        self._printed_once = True
        print(self._format_report_locked("ZYARM_RECORD_PROFILE_ONCE"), flush=True)

    def _print_interval_if_ready_locked(self, now: float) -> None:
        if self._window_start is None:
            return
        if now - self._window_start < ZYARM_PROFILE_INTERVAL_S:
            return
        if any(not self._values.get(name) for name in ("loop_ms", *_REQUIRED_METRICS)):
            return
        print(self._format_report_locked("ZYARM_RECORD_PROFILE_INTERVAL"), flush=True)
        self._clear_window_locked(now)

    def _clear_window_locked(self, now: float) -> None:
        self._values.clear()
        self._camera_values.clear()
        self._latest_master_hz = None
        self._latest_status_hz = None
        self._latest_master_gaps = None
        self._window_start = now

    def _format_report_locked(self, label: str) -> str:
        parts = [
            f"[{label}]",
            f"mode={_record_profile_mode()}",
        ]
        if label == "ZYARM_RECORD_PROFILE_INTERVAL":
            parts.append(f"window_s={ZYARM_PROFILE_INTERVAL_S:.1f}")
        else:
            parts.append(f"samples={ZYARM_PROFILE_SAMPLE_FRAMES}")
        parts.append(f"warmup={ZYARM_PROFILE_WARMUP_FRAMES}")

        # Minimal validation metrics for 20ms/50Hz acceptance
        loop_values = self._values.get("loop_ms", [])
        if loop_values:
            loop_avg = sum(loop_values) / len(loop_values)
            loop_max = max(loop_values)
            parts.append(f"loop_avg_ms={loop_avg:.1f}")
            parts.append(f"loop_max_ms={loop_max:.1f}")

        if self._latest_status_hz is not None:
            parts.append(f"status_hz={float(self._latest_status_hz):.1f}")

        return " ".join(parts)


class _TrainRealtimeProfiler:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        self._printed_once = False
        self._loop_count = 0
        self._sample_count = 0
        self._values: DefaultDict[str, list[float]] = defaultdict(list)
        self._window_start: Optional[float] = None
        self._latest_status_hz: Optional[float] = None
        self._stale_at_send_count = 0

    def enabled(self) -> bool:
        mode = _train_profile_mode()
        return mode == "interval" or (mode == "once" and not self._printed_once)

    def sample(
        self,
        *,
        loop_ms: float,
        obs_ms: float,
        policy_ms: float,
        send_ms: float,
        state_age_at_obs_ms: Any,
        state_age_at_send_ms: Any,
        control_interval_ms: float,
        state_max_age_ms: Any,
        arm: Any = None,
    ) -> None:
        mode = _train_profile_mode()
        if mode == "off" or (mode == "once" and self._printed_once):
            return

        status_hz = _read_status_hz(arm)
        now = time.perf_counter()
        with self._lock:
            self._loop_count += 1
            if self._loop_count <= ZYARM_PROFILE_WARMUP_FRAMES:
                return
            if mode == "once" and self._sample_count >= ZYARM_PROFILE_SAMPLE_FRAMES:
                self._print_once_if_ready_locked()
                return
            if self._window_start is None:
                self._window_start = now

            parsed_state_age_at_obs_ms = _parse_float(state_age_at_obs_ms)
            parsed_state_age_at_send_ms = _parse_float(state_age_at_send_ms)
            freshness_used_ms = None
            if parsed_state_age_at_obs_ms is not None and parsed_state_age_at_send_ms is not None:
                freshness_used_ms = max(0.0, parsed_state_age_at_send_ms - parsed_state_age_at_obs_ms)

            self._add_value_locked("loop_ms", loop_ms)
            self._add_value_locked("obs_ms", obs_ms)
            self._add_value_locked("policy_ms", policy_ms)
            self._add_value_locked("send_ms", send_ms)
            self._add_value_locked("state_age_at_obs_ms", parsed_state_age_at_obs_ms)
            self._add_value_locked("state_age_at_send_ms", parsed_state_age_at_send_ms)
            self._add_value_locked("freshness_used_ms", freshness_used_ms)
            self._add_value_locked("overrun_ms", max(0.0, float(loop_ms) - float(control_interval_ms)))
            if status_hz is not None:
                self._latest_status_hz = status_hz

            stale_limit_ms = _parse_float(state_max_age_ms)
            if (
                stale_limit_ms is not None
                and parsed_state_age_at_send_ms is not None
                and parsed_state_age_at_send_ms > stale_limit_ms
            ):
                self._stale_at_send_count += 1

            self._sample_count += 1
            if mode == "once":
                self._print_once_if_ready_locked()
            elif mode == "interval":
                self._print_interval_if_ready_locked(now)

    def _add_value_locked(self, name: str, value: Any) -> None:
        parsed = _parse_float(value)
        if parsed is not None:
            self._values[name].append(parsed)

    def _print_once_if_ready_locked(self) -> None:
        if self._printed_once:
            return
        if any(len(self._values[name]) < ZYARM_PROFILE_SAMPLE_FRAMES for name in _TRAIN_REQUIRED_METRICS):
            return
        self._printed_once = True
        print(self._format_report_locked("ZYARM_TRAIN_PROFILE_ONCE"), flush=True)

    def _print_interval_if_ready_locked(self, now: float) -> None:
        if self._window_start is None:
            return
        if now - self._window_start < ZYARM_PROFILE_INTERVAL_S:
            return
        if any(not self._values.get(name) for name in _TRAIN_REQUIRED_METRICS):
            return
        print(self._format_report_locked("ZYARM_TRAIN_PROFILE_INTERVAL"), flush=True)
        self._clear_window_locked(now)

    def _clear_window_locked(self, now: float) -> None:
        self._sample_count = 0
        self._values.clear()
        self._window_start = now
        self._latest_status_hz = None
        self._stale_at_send_count = 0

    def _format_report_locked(self, label: str) -> str:
        parts = [
            f"[{label}]",
            f"mode={_train_profile_mode()}",
        ]
        if label == "ZYARM_TRAIN_PROFILE_INTERVAL":
            parts.append(f"window_s={ZYARM_PROFILE_INTERVAL_S:.1f}")
        else:
            parts.append(f"samples={ZYARM_PROFILE_SAMPLE_FRAMES}")
        parts.append(f"warmup={ZYARM_PROFILE_WARMUP_FRAMES}")

        for name in _TRAIN_REPORT_METRICS:
            values = self._values.get(name, [])
            if not values:
                continue
            parts.append(
                f"{name}_p50={_percentile(values, 50):.1f}"
                f" {name}_p90={_percentile(values, 90):.1f}"
                f" {name}_p99={_percentile(values, 99):.1f}"
                f" {name}_max={max(values):.1f}"
            )
        parts.append(f"stale_at_send={self._stale_at_send_count}")
        if self._latest_status_hz is not None:
            parts.append(f"status_hz={float(self._latest_status_hz):.1f}")
        return " ".join(parts)


def _record_profile_mode() -> str:
    return _normalize_profile_mode(ZYARM_RECORD_PROFILER_MODE)


def _train_profile_mode() -> str:
    return _normalize_profile_mode(ZYARM_TRAIN_PROFILER_MODE)


def _normalize_profile_mode(value: Any) -> str:
    mode = str(value).strip().lower()
    return mode if mode in _VALID_PROFILE_MODES else "off"


def _parse_float(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _read_status_hz(arm: Any) -> Optional[float]:
    if arm is None:
        return None
    transport = getattr(arm, "transport", None)
    get_stats = getattr(transport, "get_frame_stats", None)
    if not callable(get_stats):
        return None
    try:
        stats = get_stats()
    except Exception:
        return None
    return _parse_float(getattr(stats, "status_rate_hz", None))


def _percentile(values: list[float], percentile: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = int(round((percentile / 100.0) * (len(ordered) - 1)))
    return ordered[min(max(index, 0), len(ordered) - 1)]


once_profiler = _DataQualityProfiler()
train_profiler = _TrainRealtimeProfiler()


def reset_profile_for_test() -> None:
    once_profiler.reset()
    train_profiler.reset()
