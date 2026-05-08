from __future__ import annotations

import datetime
import re
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence

from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE, Serial


ACK_RE = re.compile(r"ACK_COMPLETED:\s*CMD_ID=(\d+),\s*(SUCCESS|ERROR)")
STATUS_RE = re.compile(
    r"\[STATUS\]\s*J0:([-\d.]+)\s*J1:([-\d.]+)\s*J2:([-\d.]+)\s*J3:([-\d.]+)\s*J4:([-\d.]+)\s*J5:([-\d.]+)\s*CLAW:([-\d.]+)"
)
MASTER_DATA_RE = re.compile(
    r"\[MD\]\[(\d)\]\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]"
)
SERIAL_ERROR_PREFIX = "[ERROR]"
SERIAL_WARNING_PREFIX = "[WARN]"
SERIAL_DIAGNOSTIC_REPEAT_WINDOW_S = 1.0


class CommandId:
    RESET = 1
    JOINT_SYNC = 3
    STATUS = 6
    STATUS_REPORT = 17
    GET_STATUS_REPORT = 18
    MASTER_SLAVE = 32
    MASTER_SLAVE_STOP = 33
    JOINT_IO_FAST = 36


@dataclass(frozen=True)
class ProtocolConfig:
    port: str
    baudrate: int
    timeout_s: float
    write_timeout_s: float
    ack_timeout_s: float
    reset_rts_dtr: bool
    reset_rts_dtr_quiet_s: float
    log_serial: bool
    log_dir: str


@dataclass(frozen=True)
class StatusFrame:
    values: List[float]
    received_at: float
    raw_line: str


@dataclass(frozen=True)
class MasterDataFrame:
    frame_id: int
    values: List[float]
    received_at: float
    raw_line: str


StatusListener = Callable[[StatusFrame], None]
MasterDataListener = Callable[[MasterDataFrame], None]


class SerialProtocol:
    def __init__(self, config: ProtocolConfig, logger) -> None:
        self._config = config
        self._logger = logger
        self._serial: Optional[Serial] = None
        self._io_lock = threading.RLock()
        self._ack_cv = threading.Condition()
        self._ack_state: Dict[int, bool] = {}
        self._status_lock = threading.Lock()
        self._status_cv = threading.Condition(self._status_lock)
        self._latest_status: Optional[StatusFrame] = None
        self._latest_master_data: Optional[MasterDataFrame] = None
        self._status_listeners: List[StatusListener] = []
        self._master_data_listeners: List[MasterDataListener] = []
        self._rx_buf = bytearray()
        self._rx_stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._log_lock = threading.Lock()
        self._log_queue: List[str] = []
        self._log_stop = threading.Event()
        self._log_thread: Optional[threading.Thread] = None
        self._log_file = None
        self._diagnostic_lock = threading.Lock()
        self._last_diagnostic_line: Optional[str] = None
        self._last_diagnostic_level: Optional[str] = None
        self._last_diagnostic_at = 0.0
        self._suppressed_diagnostic_count = 0

    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open

    def add_status_listener(self, listener: StatusListener) -> None:
        self._status_listeners.append(listener)

    def add_master_data_listener(self, listener: MasterDataListener) -> None:
        self._master_data_listeners.append(listener)

    def connect(self) -> None:
        if self.is_connected:
            return
        self._serial = Serial(
            port=self._config.port,
            baudrate=self._config.baudrate,
            bytesize=EIGHTBITS,
            parity=PARITY_NONE,
            stopbits=STOPBITS_ONE,
            timeout=self._config.timeout_s,
            write_timeout=self._config.write_timeout_s,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self._serial.reset_input_buffer()
        self._serial.reset_output_buffer()
        if self._config.reset_rts_dtr:
            self._serial.setDTR(False)
            self._serial.setRTS(True)
            time.sleep(1.0)
            self._serial.setRTS(False)
            if self._config.reset_rts_dtr_quiet_s > 0.0:
                time.sleep(self._config.reset_rts_dtr_quiet_s)
        self._start_log_thread()
        self._start_rx_thread()

    def close(self) -> None:
        self._rx_stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None
        self._log_stop.set()
        if self._log_thread and self._log_thread.is_alive():
            self._log_thread.join(timeout=0.5)
        self._log_thread = None
        if self._log_file and not self._log_file.closed:
            self._log_file.close()
        self._log_file = None
        if self._serial and self._serial.is_open:
            self._serial.close()
        self._serial = None

    def send_command(
        self,
        command_id: int,
        params: Optional[Sequence[float]] = None,
        *,
        wait_ack: bool = True,
    ) -> bool:
        command = self._format_command(command_id, params)
        with self._io_lock:
            serial = self._serial
            if serial is None or not serial.is_open:
                raise RuntimeError("Serial port is not open")
            with self._ack_cv:
                self._ack_state.pop(command_id, None)
            serial.write(command.encode("utf-8"))
            if wait_ack:
                serial.flush()
            self._enqueue_log("TX", command.strip())
            if not wait_ack:
                return True
            deadline = time.perf_counter() + self._config.ack_timeout_s
            with self._ack_cv:
                while True:
                    result = self._ack_state.get(command_id)
                    if result is not None:
                        return result
                    remaining = deadline - time.perf_counter()
                    if remaining <= 0.0:
                        return False
                    self._ack_cv.wait(timeout=remaining)

    def get_latest_status(self) -> Optional[StatusFrame]:
        with self._status_lock:
            return self._latest_status

    def latest_status_received_at(self) -> float:
        with self._status_lock:
            if self._latest_status is None:
                return float("-inf")
            return self._latest_status.received_at

    def has_status_since(self, received_at: float) -> bool:
        return self.latest_status_received_at() > float(received_at)

    def wait_for_status_since(
        self,
        received_at: float,
        *,
        timeout_s: Optional[float] = None,
    ) -> Optional[StatusFrame]:
        deadline = None
        if timeout_s is not None:
            deadline = time.perf_counter() + float(timeout_s)
        with self._status_cv:
            while True:
                latest = self._latest_status
                if latest is not None and latest.received_at > float(received_at):
                    return latest
                if deadline is None:
                    self._status_cv.wait()
                    continue
                remaining = deadline - time.perf_counter()
                if remaining <= 0.0:
                    return None
                self._status_cv.wait(timeout=remaining)

    def get_latest_master_data(self) -> Optional[MasterDataFrame]:
        with self._status_lock:
            return self._latest_master_data

    def _format_command(self, command_id: int, params: Optional[Sequence[float]] = None) -> str:
        if not params:
            return f"[CMD][{command_id}]\n"
        serialized: List[str] = []
        for value in params:
            numeric = float(value)
            if numeric.is_integer():
                serialized.append(str(int(numeric)))
            else:
                serialized.append(f"{numeric:.3f}")
        return f"[CMD][{command_id}][{' '.join(serialized)}]\n"

    def _start_rx_thread(self) -> None:
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def _start_log_thread(self) -> None:
        if not self._config.log_serial:
            return
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = Path(self._config.log_dir)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"zyarm_serial_{timestamp}.log"
        self._log_file = log_path.open("a", encoding="utf-8")
        self._log_stop.clear()
        self._log_thread = threading.Thread(target=self._log_loop, daemon=True)
        self._log_thread.start()
        self._logger.info(f"Serial log file: {log_path}")

    def _log_loop(self) -> None:
        while not self._log_stop.is_set():
            time.sleep(0.05)
            self._flush_log_queue()
        self._flush_log_queue()

    def _flush_log_queue(self) -> None:
        if self._log_file is None:
            return
        with self._log_lock:
            if not self._log_queue:
                return
            lines = self._log_queue
            self._log_queue = []
        self._log_file.writelines(lines)
        self._log_file.flush()

    def _enqueue_log(self, direction: str, line: str) -> None:
        if not self._config.log_serial:
            return
        stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with self._log_lock:
            self._log_queue.append(f"[{stamp}] [{direction}] {line}\n")

    def _rx_loop(self) -> None:
        while not self._rx_stop.is_set():
            serial = self._serial
            if serial is None or not serial.is_open:
                time.sleep(0.05)
                continue
            try:
                chunk = serial.read(getattr(serial, "in_waiting", 0) or 1)
            except Exception:
                time.sleep(0.05)
                continue
            if not chunk:
                continue
            self._rx_buf.extend(chunk)
            while True:
                line_end = self._rx_buf.find(b"\n")
                if line_end < 0:
                    break
                raw_line = bytes(self._rx_buf[:line_end])
                del self._rx_buf[: line_end + 1]
                line = raw_line.decode("utf-8", errors="ignore").strip("\r\n")
                if not line or "[CMD]" in line:
                    continue
                try:
                    self._handle_rx_line(line)
                except Exception as exc:
                    self._report_rx_line_failure(line, exc)

    def _handle_rx_line(self, line: str) -> None:
        self._enqueue_log("RX", line)
        ack_match = ACK_RE.search(line)
        if ack_match:
            command_id = int(ack_match.group(1))
            ok = ack_match.group(2).upper() == "SUCCESS"
            with self._ack_cv:
                self._ack_state[command_id] = ok
                self._ack_cv.notify_all()
            return

        status_match = STATUS_RE.search(line)
        if status_match:
            frame = StatusFrame(
                values=[float(status_match.group(index)) for index in range(1, 8)],
                received_at=time.perf_counter(),
                raw_line=line,
            )
            with self._status_cv:
                self._latest_status = frame
                self._status_cv.notify_all()
            for listener in list(self._status_listeners):
                self._invoke_listener(listener, frame)
            return

        master_match = MASTER_DATA_RE.search(line)
        if master_match:
            raw_values = [master_match.group(index) for index in range(2, 9)]
            try:
                values = [float(value) for value in raw_values]
            except ValueError as exc:
                self._logger.warning(
                    f"Malformed [MD] frame on {self._config.port}, skipping line: {line!r}, "
                    f"fields={raw_values!r}, error={exc}"
                )
                return
            frame = MasterDataFrame(
                frame_id=int(master_match.group(1)),
                values=values,
                received_at=time.perf_counter(),
                raw_line=line,
            )
            with self._status_lock:
                self._latest_master_data = frame
            for listener in list(self._master_data_listeners):
                self._invoke_listener(listener, frame)
            return

        self._report_serial_diagnostic(line)

    def _report_serial_diagnostic(self, line: str) -> None:
        level = self._classify_serial_diagnostic_level(line)
        if level is None:
            return

        now = time.perf_counter()
        repeat_count = 0
        with self._diagnostic_lock:
            if (
                self._last_diagnostic_line == line
                and self._last_diagnostic_level == level
                and (now - self._last_diagnostic_at) < SERIAL_DIAGNOSTIC_REPEAT_WINDOW_S
            ):
                self._suppressed_diagnostic_count += 1
                return

            if self._last_diagnostic_line == line and self._last_diagnostic_level == level:
                repeat_count = self._suppressed_diagnostic_count

            self._last_diagnostic_line = line
            self._last_diagnostic_level = level
            self._last_diagnostic_at = now
            self._suppressed_diagnostic_count = 0

        message = f"Serial {level} on {self._config.port}: {line}"
        if repeat_count:
            message += (
                f" (repeated {repeat_count} times within "
                f"{SERIAL_DIAGNOSTIC_REPEAT_WINDOW_S:.1f}s)"
            )
        if level == "error":
            self._logger.error(message)
        else:
            self._logger.warning(message)

    def _report_rx_line_failure(self, line: str, exc: Exception) -> None:
        try:
            self._logger.warning(
                f"Serial RX line handler failed on {self._config.port}: {exc}; line={line!r}"
            )
        except Exception:
            pass

    def _classify_serial_diagnostic_level(self, line: str) -> Optional[str]:
        if line.startswith(SERIAL_ERROR_PREFIX):
            return "error"
        if line.startswith(SERIAL_WARNING_PREFIX):
            return "warning"
        return None

    def _invoke_listener(self, listener, frame) -> None:
        try:
            listener(frame)
        except Exception as exc:
            self._logger.warning(f"SerialProtocol listener failed on {self._config.port}: {exc}")
