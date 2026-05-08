# ZYArm firmware protocol notes

The SDK talks to the firmware through newline-delimited text commands:

```text
[CMD][<id>]
[CMD][<id>][<space separated params>]
```

Important command IDs:

- `CMD6 status`: active status query. The SDK sends `[CMD][6]` and waits for a fresh `[STATUS]` frame only when `query_state()` is called.
- `CMD17 status_report`: periodic background status output. It is useful for diagnostics and low-rate streams, but callers should not mix it into high-frequency control loops without treating the source explicitly.
- `CMD32 master-slave`: enters master mode. The leader arm then emits `[MD]` frames that can become teleoperation actions.
- `CMD33 master-slave stop`: leaves master mode.
- `CMD36 joint_io_fast`: fast I/O hot path. Firmware reads a joint snapshot first, then applies the target. The returned `[STATUS]` is a measured snapshot/pre-command state, not a guaranteed post-command completion state.

Response lines:

- `ACK_COMPLETED: CMD_ID=<id>, SUCCESS|ERROR` is an ACK for commands that choose to wait for ACK.
- `[STATUS] J0:<v> J1:<v> J2:<v> J3:<v> J4:<v> J5:<v> CLAW:<v>` is a firmware-unit state frame.
- `[MD][<frame_id>][<j0> <j1> <j2> <j3> <j4> <j5> <claw>]` is leader master-data output. Current firmware emits `frame_id` as a cyclic `0..9` marker, so SDK frame-gap diagnostics are weak observed events, not a complete count of firmware-side skipped or lost frames.

The no-change sentinel is `-999.9`. It is only used inside protocol/mapping layers when an `apply_mask` entry is false. Public SDK callers should pass radians for six arm joints and normalized `0.0..1.0` values for the gripper.

Angle expressions:

- Firmware expression: raw command and status values are the firmware's joint angles. These values include the hardware zero offsets and sign conventions used by the controller. For example, the physical initial pose is not necessarily all zeros in firmware `[STATUS]`.
- SDK/ROS expression: public API values are normalized for application development. In the physical initial pose, all six arm joints are represented as `0` radians. The gripper is represented as `0.0..1.0`.
- Conversion boundary: protocol parsing returns firmware values first; `mapping` converts them into SDK/ROS expression before constructing `ArmState` or `TeleopAction`. CMD36 formatting performs the inverse conversion before writing to serial.
- Documentation and examples should name the expression explicitly when raw firmware values are shown.

The transport owns serial RX in memory. ACK, STATUS and MD parsing must not depend on filesystem log files.
