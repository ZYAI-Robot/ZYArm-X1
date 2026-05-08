from __future__ import annotations

from collections import deque
import threading
import time
from typing import Callable, Deque, Dict, List, Optional, Sequence

from .config import ZyArmConfig
from .errors import TransportError
from .protocol import (
    CommandId,
    MasterDataFrame,
    StatusFrame,
    format_command,
    parse_ack,
    parse_master_data_line,
    parse_status_line,
)
from .types import ArmFrameStats


SerialFactory = Callable[..., object]
FRAME_RATE_WINDOW_S = 1.0


class SerialTransport:
    def __init__(
        self,
        config: ZyArmConfig,
        *,
        serial_factory: Optional[SerialFactory] = None,
    ) -> None:
        self.config = config
        self._serial_factory = serial_factory
        self._serial: Optional[object] = None
        self._rx_stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._io_lock = threading.RLock()
        self._ack_cv = threading.Condition()
        self._ack_state: Dict[int, bool] = {}
        self._status_cv = threading.Condition()
        self._latest_status: Optional[StatusFrame] = None
        self._latest_master_data: Optional[MasterDataFrame] = None
        self._status_sequence = 0
        self._master_sequence = 0
        self._stats_status_received = 0
        self._stats_master_data_received = 0
        self._stats_master_data_gap_count = 0
        self._stats_last_master_frame_id: Optional[int] = None
        self._status_rate_timestamps: Deque[float] = deque()
        self._master_rate_timestamps: Deque[float] = deque()
        self._rx_buffer = bytearray()

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and bool(getattr(self._serial, "is_open", True))

    def connect(self) -> None:
        if self.is_connected:
            return
        factory = self._serial_factory or self._load_pyserial_factory()
        self._serial = factory(
            port=self.config.port,
            baudrate=self.config.baudrate,
            timeout=self.config.timeout_s,
            write_timeout=self.config.write_timeout_s,
        )
        self._call_if_present("reset_input_buffer")
        self._call_if_present("reset_output_buffer")
        if self.config.reset_rts_dtr:
            self._call_if_present("setDTR", False)
            self._call_if_present("setRTS", True)
            time.sleep(1.0)
            self._call_if_present("setRTS", False)
            if self.config.reset_quiet_s > 0.0:
                time.sleep(self.config.reset_quiet_s)
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def close(self) -> None:
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None
        serial = self._serial
        self._serial = None
        if serial is not None and hasattr(serial, "close"):
            serial.close()

    def send_command(
        self,
        command_id: int,
        params: Optional[Sequence[float]] = None,
        *,
        wait_ack: bool = False,
        timeout_s: Optional[float] = None,
    ) -> bool:
        line = format_command(command_id, params)
        with self._io_lock:
            serial = self._serial
            if serial is None or not self.is_connected:
                raise TransportError("Serial port is not open")
            with self._ack_cv:
                self._ack_state.pop(int(command_id), None)
            serial.write(line.encode("utf-8"))
            if hasattr(serial, "flush"):
                serial.flush()
        if not wait_ack:
            return True
        return self.wait_for_ack(command_id, timeout_s=timeout_s)

    def wait_for_ack(self, command_id: int, *, timeout_s: Optional[float] = None) -> bool:
        deadline = time.perf_counter() + (self.config.ack_timeout_s if timeout_s is None else timeout_s)
        with self._ack_cv:
            while True:
                result = self._ack_state.get(int(command_id))
                if result is not None:
                    return result
                remaining = deadline - time.perf_counter()
                if remaining <= 0.0:
                    return False
                self._ack_cv.wait(timeout=remaining)

    @property
    def status_sequence(self) -> int:
        with self._status_cv:
            return self._status_sequence

    @property
    def master_data_sequence(self) -> int:
        with self._status_cv:
            return self._master_sequence

    def latest_status(self) -> Optional[StatusFrame]:
        with self._status_cv:
            return self._latest_status

    def wait_for_status_after(self, sequence: int, timeout_s: float) -> Optional[StatusFrame]:
        deadline = time.perf_counter() + float(timeout_s)
        with self._status_cv:
            while True:
                if self._latest_status is not None and self._latest_status.sequence > sequence:
                    return self._latest_status
                remaining = deadline - time.perf_counter()
                if remaining <= 0.0:
                    return None
                self._status_cv.wait(timeout=remaining)

    def latest_master_data(self) -> Optional[MasterDataFrame]:
        with self._status_cv:
            return self._latest_master_data

    def wait_for_master_data_after(self, sequence: int, timeout_s: float) -> Optional[MasterDataFrame]:
        deadline = time.perf_counter() + float(timeout_s)
        with self._status_cv:
            while True:
                if self._latest_master_data is not None and self._latest_master_data.sequence > sequence:
                    return self._latest_master_data
                remaining = deadline - time.perf_counter()
                if remaining <= 0.0:
                    return None
                self._status_cv.wait(timeout=remaining)

    def get_frame_stats(self) -> ArmFrameStats:
        with self._status_cv:
            self._prune_rate_windows(time.perf_counter())
            return ArmFrameStats(
                master_data_received=self._stats_master_data_received,
                master_data_gap_count=self._stats_master_data_gap_count,
                master_data_rate_hz=float(len(self._master_rate_timestamps)),
                status_received=self._stats_status_received,
                status_rate_hz=float(len(self._status_rate_timestamps)),
            )

    def reset_frame_stats(self) -> None:
        with self._status_cv:
            self._stats_status_received = 0
            self._stats_master_data_received = 0
            self._stats_master_data_gap_count = 0
            self._stats_last_master_frame_id = None
            self._status_rate_timestamps.clear()
            self._master_rate_timestamps.clear()

    def feed_line_for_test(self, line: str) -> None:
        self._handle_rx_line(line.strip())

    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set():
            serial = self._serial
            if serial is None:
                time.sleep(0.01)
                continue
            try:
                in_waiting = int(getattr(serial, "in_waiting", 0) or 1)
                chunk = serial.read(in_waiting)
            except Exception:
                time.sleep(0.01)
                continue
            if not chunk:
                continue
            self._rx_buffer.extend(chunk)
            while True:
                line_end = self._rx_buffer.find(b"\n")
                if line_end < 0:
                    break
                raw = bytes(self._rx_buffer[:line_end])
                del self._rx_buffer[: line_end + 1]
                line = raw.decode("utf-8", errors="ignore").strip("\r\n")
                if line:
                    self._handle_rx_line(line)

    def _handle_rx_line(self, line: str) -> None:
        ack = parse_ack(line)
        if ack is not None:
            with self._ack_cv:
                self._ack_state[ack.command_id] = ack.success
                self._ack_cv.notify_all()
            return

        with self._status_cv:
            next_status_sequence = self._status_sequence + 1
            status = parse_status_line(line, sequence=next_status_sequence)
            if status is not None:
                self._status_sequence = next_status_sequence
                self._latest_status = status
                self._stats_status_received += 1
                self._status_rate_timestamps.append(status.received_at)
                self._prune_rate_windows(status.received_at)
                self._status_cv.notify_all()
                return

            next_master_sequence = self._master_sequence + 1
            master = parse_master_data_line(line, sequence=next_master_sequence)
            if master is not None:
                self._master_sequence = next_master_sequence
                self._latest_master_data = master
                self._record_master_data_stats(master)
                self._status_cv.notify_all()

    def _record_master_data_stats(self, frame: MasterDataFrame) -> None:
        frame_id = int(frame.frame_id) % 10
        if self._stats_last_master_frame_id is not None:
            self._stats_master_data_gap_count += (
                frame_id - self._stats_last_master_frame_id - 1
            ) % 10
        self._stats_last_master_frame_id = frame_id
        self._stats_master_data_received += 1
        self._master_rate_timestamps.append(frame.received_at)
        self._prune_rate_windows(frame.received_at)

    def _prune_rate_windows(self, now: float) -> None:
        cutoff = now - FRAME_RATE_WINDOW_S
        while self._status_rate_timestamps and self._status_rate_timestamps[0] < cutoff:
            self._status_rate_timestamps.popleft()
        while self._master_rate_timestamps and self._master_rate_timestamps[0] < cutoff:
            self._master_rate_timestamps.popleft()

    def _call_if_present(self, name: str, *args) -> None:
        serial = self._serial
        if serial is not None and hasattr(serial, name):
            getattr(serial, name)(*args)

    def _load_pyserial_factory(self) -> SerialFactory:
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise TransportError("pyserial is required for real serial transport") from exc
        return serial.Serial


class MemoryTransport(SerialTransport):
    """Test/demo transport with no OS serial dependency."""

    def __init__(self, config: Optional[ZyArmConfig] = None, *, auto_ack: bool = True) -> None:
        super().__init__(config or ZyArmConfig(port="memory"))
        self.written_lines: List[str] = []
        self._connected = False
        self._auto_ack = auto_ack

    @property
    def is_connected(self) -> bool:
        return self._connected

    def connect(self) -> None:
        self._connected = True

    def close(self) -> None:
        self._connected = False

    def send_command(
        self,
        command_id: int,
        params: Optional[Sequence[float]] = None,
        *,
        wait_ack: bool = False,
        timeout_s: Optional[float] = None,
    ) -> bool:
        if not self._connected:
            raise TransportError("Memory transport is not connected")
        self.written_lines.append(format_command(command_id, params))
        if self._auto_ack:
            with self._ack_cv:
                self._ack_state[int(command_id)] = True
                self._ack_cv.notify_all()
        if wait_ack:
            return self.wait_for_ack(command_id, timeout_s=timeout_s)
        return True
