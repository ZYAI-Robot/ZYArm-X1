import inspect

import zyarm_hardware.protocol as protocol_module
from zyarm_hardware.protocol import ProtocolConfig, SerialProtocol


class FakeLogger:
    def __init__(self) -> None:
        self.errors = []
        self.warnings = []
        self.infos = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    def info(self, message: str) -> None:
        self.infos.append(message)


class CallsiteStrictLogger(FakeLogger):
    def __init__(self) -> None:
        super().__init__()
        self._severities_by_callsite = {}

    def error(self, message: str) -> None:
        self._record("error", message)

    def warning(self, message: str) -> None:
        self._record("warning", message)

    def _record(self, severity: str, message: str) -> None:
        frame = inspect.currentframe()
        assert frame is not None
        caller = frame.f_back.f_back
        callsite = (caller.f_code.co_filename, caller.f_lineno)
        previous = self._severities_by_callsite.setdefault(callsite, severity)
        if previous != severity:
            raise ValueError("Logger severity cannot be changed between calls.")
        if severity == "error":
            self.errors.append(message)
        else:
            self.warnings.append(message)


def _config() -> ProtocolConfig:
    return ProtocolConfig(
        port="/dev/null",
        baudrate=230400,
        timeout_s=0.1,
        write_timeout_s=0.1,
        ack_timeout_s=0.2,
        reset_rts_dtr=False,
        reset_rts_dtr_quiet_s=3.0,
        log_serial=False,
        log_dir="/tmp",
    )


def test_status_listener_receives_parsed_status_frame():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)
    received = []
    protocol.add_status_listener(lambda frame: received.append(frame.values))

    protocol._handle_rx_line("[STATUS] J0:1 J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7")

    assert received == [[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]]
    assert protocol.get_latest_status().values == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]


def test_has_status_since_tracks_latest_status_timestamp():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    before = protocol.latest_status_received_at()
    protocol._handle_rx_line("[STATUS] J0:1 J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7")

    assert before == float("-inf")
    assert protocol.has_status_since(before) is True
    assert protocol.has_status_since(protocol.latest_status_received_at()) is False


def test_wait_for_status_since_returns_fresh_status_frame():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    before = protocol.latest_status_received_at()
    protocol._handle_rx_line("[STATUS] J0:1 J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7")

    frame = protocol.wait_for_status_since(before, timeout_s=0.01)

    assert frame is not None
    assert frame.values == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]


def test_wait_for_status_since_times_out_without_fresh_frame():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    frame = protocol.wait_for_status_since(protocol.latest_status_received_at(), timeout_s=0.0)

    assert frame is None


def test_connect_waits_after_rts_dtr_reset(monkeypatch):
    sleep_calls = []
    serial_instances = []

    class FakeSerial:
        def __init__(self, *args, **kwargs) -> None:
            self.is_open = True
            self.dtr_values = []
            self.rts_values = []
            serial_instances.append(self)

        def reset_input_buffer(self) -> None:
            pass

        def reset_output_buffer(self) -> None:
            pass

        def setDTR(self, value: bool) -> None:
            self.dtr_values.append(value)

        def setRTS(self, value: bool) -> None:
            self.rts_values.append(value)

        def close(self) -> None:
            self.is_open = False

    monkeypatch.setattr(protocol_module, "Serial", FakeSerial)
    monkeypatch.setattr(protocol_module.time, "sleep", lambda seconds: sleep_calls.append(seconds))

    logger = FakeLogger()
    config = ProtocolConfig(
        port="/dev/ttyUSB0",
        baudrate=230400,
        timeout_s=0.1,
        write_timeout_s=0.1,
        ack_timeout_s=0.2,
        reset_rts_dtr=True,
        reset_rts_dtr_quiet_s=3.0,
        log_serial=False,
        log_dir="/tmp",
    )
    protocol = SerialProtocol(config, logger)
    monkeypatch.setattr(protocol, "_start_log_thread", lambda: None)
    monkeypatch.setattr(protocol, "_start_rx_thread", lambda: None)

    protocol.connect()

    assert len(serial_instances) == 1
    assert serial_instances[0].dtr_values == [False]
    assert serial_instances[0].rts_values == [True, False]
    assert sleep_calls == [1.0, 3.0]


def test_malformed_master_data_logs_and_skips_frame():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    protocol._handle_rx_line("[MD][3][83.32.10 1 2 3 4 5 6]")

    assert protocol.get_latest_master_data() is None
    assert logger.warnings
    assert "Malformed [MD] frame" in logger.warnings[0]
    assert "83.32.10" in logger.warnings[0]


def test_master_data_listener_receives_good_frame():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)
    received = []
    protocol.add_master_data_listener(lambda frame: received.append((frame.frame_id, frame.values)))

    protocol._handle_rx_line("[MD][4][10 20 30 40 50 60 70]")

    assert received == [(4, [10.0, 20.0, 30.0, 40.0, 50.0, 60.0, 70.0])]
    assert protocol.get_latest_master_data().frame_id == 4


def test_error_like_rx_line_is_logged_to_terminal():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    protocol._handle_rx_line("[ERROR][ARM_ROBOT] Master arm unload mode failed")

    assert logger.errors == [
        "Serial error on /dev/null: [ERROR][ARM_ROBOT] Master arm unload mode failed"
    ]
    assert not logger.warnings


def test_warning_like_rx_line_is_logged_to_terminal():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    protocol._handle_rx_line("[WARN][TRANSPORT] transport_stats CTRL_DROP=1 STREAM_DROP=0")

    assert logger.warnings == [
        "Serial warning on /dev/null: [WARN][TRANSPORT] transport_stats CTRL_DROP=1 STREAM_DROP=0"
    ]
    assert not logger.errors


def test_mixed_diagnostic_severities_use_stable_logger_call_sites():
    logger = CallsiteStrictLogger()
    protocol = SerialProtocol(_config(), logger)

    protocol._handle_rx_line("[ERROR][ARM_ROBOT] Master arm unload mode failed")
    protocol._handle_rx_line("[WARN][TRANSPORT] transport_stats CTRL_DROP=1 STREAM_DROP=0")

    assert logger.errors == [
        "Serial error on /dev/null: [ERROR][ARM_ROBOT] Master arm unload mode failed"
    ]
    assert logger.warnings == [
        "Serial warning on /dev/null: [WARN][TRANSPORT] transport_stats CTRL_DROP=1 STREAM_DROP=0"
    ]


def test_rx_loop_keeps_processing_after_line_handler_failure(monkeypatch):
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)
    handled = []

    class FakeSerial:
        is_open = True
        in_waiting = 0

        def read(self, _size):
            protocol._rx_stop.set()
            return b"bad\nok\n"

    def handle(line):
        handled.append(line)
        if line == "bad":
            raise RuntimeError("boom")

    protocol._serial = FakeSerial()
    monkeypatch.setattr(protocol, "_handle_rx_line", handle)

    protocol._rx_loop()

    assert handled == ["bad", "ok"]
    assert logger.warnings == [
        "Serial RX line handler failed on /dev/null: boom; line='bad'"
    ]


def test_transport_stats_response_is_not_mistaken_for_error_log():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    protocol._handle_rx_line(
        "TRANSPORT_STATS:RX_OK=1,RX_OVERFLOW=0,RX_UART_ERROR=0,TX_DMA_COMPLETE=1,"
        "TX_CTRL_DROP=0,TX_STREAM_DROP=0,TX_TRUNCATE=0,TX_DMA_ERROR=0,"
        "TX_RESERVE_CANCEL=0,TX_DIAG_EMIT=0"
    )

    assert not logger.errors
    assert not logger.warnings


def test_non_prefixed_failed_line_is_not_mistaken_for_error_log():
    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    protocol._handle_rx_line("Inverse kinematics failed, location unreachable.")

    assert not logger.errors
    assert not logger.warnings


def test_repeated_same_error_line_is_rate_limited(monkeypatch):
    perf_counter_values = iter([0.0, 0.2, 1.3])
    monkeypatch.setattr(protocol_module.time, "perf_counter", lambda: next(perf_counter_values))

    logger = FakeLogger()
    protocol = SerialProtocol(_config(), logger)

    protocol._handle_rx_line("[ERROR][STATUS_REPORT] status_report stop timeout")
    protocol._handle_rx_line("[ERROR][STATUS_REPORT] status_report stop timeout")
    protocol._handle_rx_line("[ERROR][STATUS_REPORT] status_report stop timeout")

    assert logger.errors == [
        "Serial error on /dev/null: [ERROR][STATUS_REPORT] status_report stop timeout",
        "Serial error on /dev/null: [ERROR][STATUS_REPORT] status_report stop timeout "
        "(repeated 1 times within 1.0s)",
    ]
