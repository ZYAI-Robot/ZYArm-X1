# ZYArm SDK API design

`zyarm_sdk` is a standalone SDK for Python/C++ programs, examples, teaching code and future LeRobot native adapters. It does not depend on ROS, MoveIt, ros2_control or LeRobot.

Core public concepts:

- `ZyArmConfig`: serial port, baudrate, timeouts, mapping and safety configuration. The default baudrate is `230400`.
- `ZyArm`: single-arm lifecycle and commands: `connect`, `close`, `get_latest_state`, `query_state`, `fast_io`, `reset`, `stop`.
- `ArmState`: public-unit state with source, timestamp, sequence and `age_ms`.
- `FastIoResult`: CMD36 dispatch result with optional measured snapshot.
- `TeleopAction`: leader action from `[MD]`, with timestamp/source metadata.
- `ArmFrameStats`: SDK-observed RX health snapshot for parsed `[MD]` and `[STATUS]` frames. Frame-gap diagnostics are estimated from firmware frame IDs and are intended for teleoperation quality monitoring, not strict transport auditing.
- `ZyArmTeleopPair`: explicit leader/follower step mode and opt-in automatic demo follow mode.

State API naming:

- `get_latest_state(max_age_ms=None)` reads the in-memory cache and does not write a serial command.
- `query_state(timeout_ms=...)` sends CMD6 and waits for a fresh `[STATUS]`.
- `fast_io(..., wait_state=False)` sends CMD36. It does not wait by default. If `wait_state=True`, any returned state is labeled as a CMD36 measured snapshot/pre-command state.

Angle and unit semantics:

- Firmware angle expression: values in firmware `[STATUS]` and `[MD]` frames are hardware-facing joint angles. They include firmware zero offsets, sign conventions and the gripper command range.
- SDK/ROS angle expression: values exposed by `ZyArm`, `ArmState`, `TeleopAction` and high-level examples are user-facing values. In the arm's initial pose, all six arm joints are `0` radians, and the gripper is normalized as `0.0..1.0`.
- `mapping` is the only layer that should convert between these two expressions. Application code, examples and future LeRobot adapters should use SDK/ROS expression unless they are explicitly implementing protocol-level tools.
- When debugging raw firmware logs, do not compare `[STATUS]` degrees directly with SDK joint values. Convert through the configured `MappingConfig` first.

Performance defaults:

- The hot path writes one CMD36 line and returns.
- Automatic teleoperation follow is event-triggered by fresh `[MD]` frames. The SDK does not periodically resample cached leader actions.
- `TeleopConfig.leader_hz` configures the leader firmware's `[MD]` output frequency. There is no separate SDK-side follow sampling frequency.
- `ZyArm.get_frame_stats()` returns a snapshot of parsed `[MD]` and `[STATUS]` RX counters and approximate 1-second rates; `reset_frame_stats()` clears those counters without resetting transport sequence numbers.
- `ZyArmTeleopPair.start_step_mode()` and `start_auto_follow()` automatically start the follower in firmware slave filter mode before starting the leader, so SDK and LeRobot teleoperation use the same master-slave execution semantics.
- Per-frame console/file logging is not part of the default control loop.
- One serial connection has one RX owner that updates in-memory ACK/status/action caches.
- Ordinary ACK waits default to 1 s and are used for configuration or mode-switch commands, so serial/baudrate/protocol errors surface quickly.
- Action-completion ACK waits default to 10 s and are used for `reset()`, `move_ik()` and synchronous gripper commands. These ACKs mean the firmware reported motion completion, not just serial write success.
- Recorded-action playback ACK waits default to 190 s, covering the 3-minute maximum recorded action plus a small communication/finish margin.
- Explicit status/action waits must carry explicit timeouts.

Platform policy:

- Python uses `pyserial` for Linux and Windows ports.
- C++ hides POSIX and Win32 serial differences behind `transport`.
- Windows C++ starts with blocking read/write plus explicit timeouts; overlapped I/O is a later optimization only if measured performance requires it.

C++ examples:

```bash
cmake -S software/zyarm_sdk/cpp -B /tmp/zyarm_sdk_cpp_build -DZYARM_SDK_BUILD_EXAMPLES=ON
cmake --build /tmp/zyarm_sdk_cpp_build -j
```

The example targets are `read_state`, `fast_io_loop` and `teleop_step`.
