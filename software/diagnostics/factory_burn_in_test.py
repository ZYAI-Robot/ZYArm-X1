#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from sdk_helpers import create_arm, ok
from zyarm_sdk import ArmState, ZyArm, ZyArmConfig
from zyarm_sdk.protocol import CommandId
from zyarm_sdk.transport import MemoryTransport


TargetPose = Tuple[float, float, float, float, float, float]

DEFAULT_TARGET_POSES: List[TargetPose] = [
    (200, 0, 100, 0, 0, 0),
    (200, 0, -50, 0, 0, 0),
    (200, 0, 100, 0, 0, 0),
    (200, 100, 100, 0, 0, 0),
    (200, 100, -50, 0, 0, 0),
    (200, 100, 100, 0, 0, 0),
    (200, 0, 100, 0, 0, 0),
    (200, 0, 100, -50, 0, 0),
    (200, 0, 100, 50, 0, 0),
    (400, 0, 0, 0, 0, 0),
    (200, 0, 0, 0, 0, 0),
]
REQUIRED_SERVO_IDS = tuple(range(1, 10))


def default_targets() -> List[Tuple[str, TargetPose]]:
    return [
        (f"P{index}", pose)
        for index, pose in enumerate(DEFAULT_TARGET_POSES, start=1)
    ]


class BurnInFailure(RuntimeError):
    pass


@dataclass
class ServoTempStats:
    start_c: Optional[float] = None
    max_c: Optional[float] = None
    end_c: Optional[float] = None
    samples: int = 0
    read_failures: int = 0

    def update(self, value: float) -> None:
        value = float(value)
        if self.start_c is None:
            self.start_c = value
        self.end_c = value
        self.max_c = value if self.max_c is None else max(self.max_c, value)
        self.samples += 1

    def fail_read(self) -> None:
        self.read_failures += 1

    def to_report(self) -> Dict[str, Optional[float]]:
        rise = None
        if self.start_c is not None and self.end_c is not None:
            rise = self.end_c - self.start_c
        return {
            "start_c": None if self.start_c is None else round(self.start_c, 3),
            "max_c": None if self.max_c is None else round(self.max_c, 3),
            "end_c": None if self.end_c is None else round(self.end_c, 3),
            "temp_rise_c": None if rise is None else round(rise, 3),
            "samples": self.samples,
            "read_failures": self.read_failures,
        }


class DryRunFirmwareTransport(MemoryTransport):
    """Memory transport plus minimal firmware responses for report-format dry runs."""

    def __init__(self, config: ZyArmConfig) -> None:
        super().__init__(config)
        self._temperature_sample = 0

    def send_command(self, command_id, params=None, *, wait_ack=False, timeout_s=None):
        result = super().send_command(command_id, params, wait_ack=wait_ack, timeout_s=timeout_s)
        if int(command_id) == int(CommandId.STATUS):
            if params and float(params[0]) == 1.0:
                self._temperature_sample += 1
                base = 30.0 + min(self._temperature_sample, 20) * 0.1
                fields = " ".join(
                    f"S{servo_id}:{base + servo_id * 0.1:.1f}"
                    for servo_id in REQUIRED_SERVO_IDS
                )
                self.feed_line_for_test(f"[SERVO_TEMP] {fields}")
            else:
                self.feed_line_for_test(
                    "[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:50"
                )
        return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ZYArm factory short burn-in test")
    parser.add_argument("--port", required=True, help="Serial port, for example COM3 or /dev/ttyUSB0")
    parser.add_argument("--sn", required=True, help="Robot serial number")
    parser.add_argument("--baudrate", type=int, default=230_400)
    parser.add_argument("--duration-min", type=float, default=15.0)
    parser.add_argument("--report-dir", default=str(Path(__file__).resolve().parent / "reports"))
    parser.add_argument("--operator", default="")
    parser.add_argument("--ambient-temp-c", type=float, default=25.0)
    parser.add_argument("--point-delay-s", type=float, default=1.0)
    parser.add_argument("--temp-warn-c", type=float, default=60.0)
    parser.add_argument("--temp-fail-c", type=float, default=70.0)
    parser.add_argument("--temp-sample-interval-s", type=float, default=20.0)
    parser.add_argument("--temp-timeout-ms", type=float, default=1000.0)
    parser.add_argument("--state-timeout-ms", type=float, default=1000.0)
    parser.add_argument("--max-temp-read-failures", type=int, default=3)
    parser.add_argument("--home-drift-threshold-rad", type=float, default=0.05)
    parser.add_argument("--serial-log-path", default=None)
    parser.add_argument("--no-serial-log", action="store_true")
    parser.add_argument("--serial-log-flush-interval-s", type=float, default=1.0)
    parser.add_argument("--dry-run", action="store_true", help="Run without hardware using SDK memory transport")
    args = parser.parse_args()

    if args.duration_min <= 0:
        parser.error("--duration-min must be > 0")
    if args.temp_sample_interval_s <= 0:
        parser.error("--temp-sample-interval-s must be > 0")
    if args.temp_fail_c <= 0:
        parser.error("--temp-fail-c must be > 0")
    if args.temp_warn_c >= args.temp_fail_c:
        parser.error("--temp-warn-c must be lower than --temp-fail-c")
    if args.point_delay_s < 0:
        parser.error("--point-delay-s must be >= 0")
    if args.serial_log_flush_interval_s < 0:
        parser.error("--serial-log-flush-interval-s must be >= 0")
    if args.max_temp_read_failures <= 0:
        parser.error("--max-temp-read-failures must be > 0")
    if args.home_drift_threshold_rad <= 0:
        parser.error("--home-drift-threshold-rad must be > 0")
    return args


def make_arm(args: argparse.Namespace) -> ZyArm:
    if args.dry_run:
        config = ZyArmConfig(port="memory", baudrate=args.baudrate)
        return ZyArm(config, transport=DryRunFirmwareTransport(config)).connect()
    return create_arm(args.port, baudrate=args.baudrate)


def iso_now() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def write_csv_row(
    writer: csv.DictWriter,
    start_time: float,
    stage: str,
    target: str = "",
    action_result: str = "",
    status: Optional[ArmState] = None,
    temperatures: Optional[Dict[int, float]] = None,
    note: str = "",
) -> None:
    writer.writerow(
        {
            "time": iso_now(),
            "elapsed_s": f"{time.perf_counter() - start_time:.3f}",
            "stage": stage,
            "target": target,
            "action_result": action_result,
            "status": "" if status is None else json.dumps(list(status.positions)),
            "temperatures_c": "" if temperatures is None else json.dumps(temperatures, sort_keys=True),
            "note": note,
        }
    )


def sample_temperatures(
    arm: ZyArm,
    args: argparse.Namespace,
    stats: Dict[int, ServoTempStats],
    writer: csv.DictWriter,
    start_time: float,
    warnings: List[Dict[str, object]],
    stage: str,
    target: str = "",
) -> Tuple[bool, Optional[str]]:
    frame = arm.query_servo_temperatures(timeout_ms=args.temp_timeout_ms)
    if frame is None:
        for item in stats.values():
            item.fail_read()
        write_csv_row(writer, start_time, stage, target, "TEMP_TIMEOUT", note="temperature query timeout")
        return False, None

    missing = [servo_id for servo_id in REQUIRED_SERVO_IDS if servo_id not in frame.temperatures_c]
    if missing:
        for servo_id in missing:
            stats[servo_id].fail_read()
        note = f"missing servo temperatures: {missing}"
        write_csv_row(
            writer,
            start_time,
            stage,
            target,
            "TEMP_INCOMPLETE",
            note=f"{note}; raw={frame.raw_line}",
        )
        return False, note

    for servo_id in REQUIRED_SERVO_IDS:
        stats[servo_id].update(frame.temperatures_c[servo_id])

    write_csv_row(
        writer,
        start_time,
        stage,
        target,
        "TEMP_OK",
        temperatures={servo_id: frame.temperatures_c[servo_id] for servo_id in REQUIRED_SERVO_IDS},
    )

    fail_hits = [
        (servo_id, frame.temperatures_c[servo_id])
        for servo_id in REQUIRED_SERVO_IDS
        if frame.temperatures_c[servo_id] >= args.temp_fail_c
    ]
    if fail_hits:
        servo_id, value = fail_hits[0]
        return True, f"servo S{servo_id} temperature {value:.1f}C reached fail threshold"

    for servo_id in REQUIRED_SERVO_IDS:
        value = frame.temperatures_c[servo_id]
        if value >= args.temp_warn_c:
            warnings.append(
                {
                    "time": iso_now(),
                    "servo_id": servo_id,
                    "temperature_c": value,
                    "message": "temperature warn threshold reached",
                }
            )
    return True, None


def max_home_drift(cold: ArmState, hot: ArmState) -> Tuple[float, int]:
    deltas = [abs(float(hot.positions[index]) - float(cold.positions[index])) for index in range(6)]
    max_delta = max(deltas)
    return max_delta, deltas.index(max_delta)


def build_report(
    args: argparse.Namespace,
    result: str,
    report_paths: Dict[str, Optional[str]],
    start_iso: str,
    end_iso: str,
    start_time: float,
    move_success: int,
    move_failures: int,
    warnings: List[Dict[str, object]],
    failure_reasons: List[str],
    temp_stats: Dict[int, ServoTempStats],
    cold_home: Optional[ArmState],
    hot_home: Optional[ArmState],
    home_drift: Optional[Dict[str, object]],
) -> Dict[str, object]:
    return {
        "result": result,
        "sn": args.sn,
        "port": args.port,
        "baudrate": args.baudrate,
        "operator": args.operator,
        "dry_run": bool(args.dry_run),
        "started_at": start_iso,
        "ended_at": end_iso,
        "duration_s": round(time.perf_counter() - start_time, 3),
        "config": {
            "duration_min": args.duration_min,
            "ambient_temp_c": args.ambient_temp_c,
            "point_delay_s": args.point_delay_s,
            "temp_warn_c": args.temp_warn_c,
            "temp_fail_c": args.temp_fail_c,
            "temp_sample_interval_s": args.temp_sample_interval_s,
            "max_temp_read_failures": args.max_temp_read_failures,
            "home_drift_threshold_rad": args.home_drift_threshold_rad,
            "serial_log_enabled": not args.no_serial_log,
            "serial_log_flush_interval_s": args.serial_log_flush_interval_s,
        },
        "paths": report_paths,
        "motion": {
            "success_count": move_success,
            "failure_count": move_failures,
        },
        "warnings": warnings,
        "failure_reasons": failure_reasons,
        "temperature_summary": {
            f"S{servo_id}": temp_stats[servo_id].to_report()
            for servo_id in REQUIRED_SERVO_IDS
        },
        "home_check": {
            "cold": None if cold_home is None else list(cold_home.positions),
            "hot": None if hot_home is None else list(hot_home.positions),
            "drift": home_drift,
        },
    }


def run() -> int:
    args = parse_args()
    run_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.report_dir).resolve() / f"{args.sn}_{run_stamp}"
    run_dir.mkdir(parents=True, exist_ok=True)
    json_path = run_dir / "factory_burn_in_report.json"
    csv_path = run_dir / "factory_burn_in_samples.csv"
    serial_log_path = None
    if not args.no_serial_log:
        serial_log_path = Path(args.serial_log_path).resolve() if args.serial_log_path else run_dir / "serial.log"
        serial_log_path.parent.mkdir(parents=True, exist_ok=True)

    temp_stats = {servo_id: ServoTempStats() for servo_id in REQUIRED_SERVO_IDS}
    warnings: List[Dict[str, object]] = []
    failure_reasons: List[str] = []
    move_success = 0
    move_failures = 0
    cold_home: Optional[ArmState] = None
    hot_home: Optional[ArmState] = None
    home_drift: Optional[Dict[str, object]] = None
    result = "PASS"
    arm: Optional[ZyArm] = None
    start_iso = iso_now()
    start_time = time.perf_counter()

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "time",
                "elapsed_s",
                "stage",
                "target",
                "action_result",
                "status",
                "temperatures_c",
                "note",
            ],
        )
        writer.writeheader()

        try:
            arm = make_arm(args)
            if serial_log_path is not None:
                arm.enable_serial_log(
                    serial_log_path,
                    include_tx=True,
                    include_rx=True,
                    flush_interval_s=args.serial_log_flush_interval_s,
                )

            reset_result = arm.reset()
            write_csv_row(writer, start_time, "cold_reset", action_result=str(reset_result.accepted))
            if not ok(reset_result):
                raise BurnInFailure("cold reset failed")

            cold_home = arm.query_state(timeout_ms=args.state_timeout_ms)
            write_csv_row(writer, start_time, "cold_home", status=cold_home)
            if cold_home is None:
                raise BurnInFailure("cold HOME state query failed")

            consecutive_temp_failures = 0
            sample_ok, sample_failure = sample_temperatures(
                arm, args, temp_stats, writer, start_time, warnings, "initial_temperature"
            )
            if not sample_ok:
                consecutive_temp_failures += 1
                if sample_failure:
                    raise BurnInFailure(sample_failure)
            elif sample_failure:
                raise BurnInFailure(sample_failure)

            deadline = start_time + args.duration_min * 60.0
            next_temp_sample = time.perf_counter() + args.temp_sample_interval_s

            while time.perf_counter() < deadline:
                for point_name, target in default_targets():
                    if time.perf_counter() >= deadline:
                        break
                    motion = arm.move_ik(*target)
                    write_csv_row(
                        writer,
                        start_time,
                        "move",
                        target=point_name,
                        action_result=str(motion.accepted),
                        note=str(target),
                    )
                    if not ok(motion):
                        move_failures += 1
                        raise BurnInFailure(f"move_ik failed at {point_name}")
                    move_success += 1

                    if args.point_delay_s > 0:
                        time.sleep(min(args.point_delay_s, max(0.0, deadline - time.perf_counter())))

                    if time.perf_counter() >= next_temp_sample:
                        sample_ok, sample_failure = sample_temperatures(
                            arm, args, temp_stats, writer, start_time, warnings, "temperature", point_name
                        )
                        if sample_ok:
                            consecutive_temp_failures = 0
                        else:
                            consecutive_temp_failures += 1
                            if sample_failure:
                                raise BurnInFailure(sample_failure)
                            if consecutive_temp_failures >= args.max_temp_read_failures:
                                raise BurnInFailure(
                                    f"temperature query failed {consecutive_temp_failures} times consecutively"
                                )
                        if sample_failure:
                            raise BurnInFailure(sample_failure)
                        while next_temp_sample <= time.perf_counter():
                            next_temp_sample += args.temp_sample_interval_s

            hot_reset = arm.reset()
            write_csv_row(writer, start_time, "hot_reset", action_result=str(hot_reset.accepted))
            if not ok(hot_reset):
                raise BurnInFailure("hot reset failed")

            hot_home = arm.query_state(timeout_ms=args.state_timeout_ms)
            write_csv_row(writer, start_time, "hot_home", status=hot_home)
            if hot_home is None:
                raise BurnInFailure("hot HOME state query failed")

            drift, joint_index = max_home_drift(cold_home, hot_home)
            home_drift = {
                "max_joint_delta_rad": drift,
                "joint_index": joint_index,
                "threshold_rad": args.home_drift_threshold_rad,
            }
            if drift > args.home_drift_threshold_rad:
                raise BurnInFailure(
                    f"hot HOME drift {drift:.4f} rad exceeds threshold at joint {joint_index}"
                )

        except KeyboardInterrupt:
            result = "INTERRUPTED"
            failure_reasons.append("operator interrupted test")
        except BurnInFailure as exc:
            result = "FAIL"
            failure_reasons.append(str(exc))
            if arm is not None:
                try:
                    arm.stop()
                except Exception:
                    pass
        except Exception as exc:
            result = "FAIL"
            failure_reasons.append(f"unexpected error: {exc}")
            if arm is not None:
                try:
                    arm.stop()
                except Exception:
                    pass
        finally:
            if arm is not None:
                try:
                    arm.flush_serial_log()
                    arm.disable_serial_log()
                    arm.close()
                except Exception:
                    pass

    report_paths = {
        "json": str(json_path),
        "csv": str(csv_path),
        "serial_log": None if serial_log_path is None else str(serial_log_path),
    }
    report = build_report(
        args,
        result,
        report_paths,
        start_iso,
        iso_now(),
        start_time,
        move_success,
        move_failures,
        warnings,
        failure_reasons,
        temp_stats,
        cold_home,
        hot_home,
        home_drift,
    )
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"Result: {result}")
    print(f"JSON report: {json_path}")
    print(f"CSV samples: {csv_path}")
    if serial_log_path is not None:
        print(f"Serial log: {serial_log_path}")
    if failure_reasons:
        print("Failure reasons:")
        for reason in failure_reasons:
            print(f"- {reason}")
    return 0 if result == "PASS" else 1


if __name__ == "__main__":
    sys.exit(run())
