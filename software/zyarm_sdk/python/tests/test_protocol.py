import math

from zyarm_sdk.protocol import (
    CommandId,
    format_command,
    format_joint_io_fast_command,
    parse_ack,
    parse_master_data_line,
    parse_servo_temperature_line,
    parse_status_line,
)


def test_format_commands() -> None:
    assert format_command(CommandId.STATUS) == "[CMD][6]\n"
    assert (
        format_joint_io_fast_command([0, -180, 90, -999.9, 50, 1.25, 100])
        == "[CMD][36][0 -180 90 -999.900 50 1.250 100]\n"
    )


def test_parse_ack_status_and_master_data() -> None:
    ack = parse_ack("ACK_COMPLETED: CMD_ID=36, SUCCESS")
    assert ack is not None
    assert ack.command_id == 36
    assert ack.success is True

    status = parse_status_line("[STATUS] J0:1 J1:-2.5 J2:3 J3:4.25 J4:5 J5:6 CLAW:7.5")
    assert status is not None
    assert status.values == [1, -2.5, 3, 4.25, 5, 6, 7.5]

    md = parse_master_data_line("[MD][4][10 20 30 40 50 60 70]")
    assert md is not None
    assert md.frame_id == 4
    assert md.values[-1] == 70


def test_invalid_lines_do_not_parse() -> None:
    assert parse_status_line("[STATUS] J0:bad J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7") is None
    assert parse_master_data_line("[MD][4][10 20 30 40 50 60]") is None
    assert parse_servo_temperature_line("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:50") is None


def test_parse_servo_temperatures() -> None:
    frame = parse_servo_temperature_line(
        "[SERVO_TEMP] S1:32 S2:30.5 S3:29 S4:28 S5:27 S6:29 S7:29 S8:29 S9:30",
        sequence=7,
        received_at=1.25,
    )
    assert frame is not None
    assert frame.sequence == 7
    assert frame.received_at == 1.25
    assert frame.temperatures_c[1] == 32
    assert frame.temperatures_c[2] == 30.5
    assert frame.temperatures_c[9] == 30

    wrapped = parse_servo_temperature_line("ACK_RESPONSE: CMD_ID=6, [SERVO_TEMP] S1:32 S2:30 S3:29")
    assert wrapped is not None
    assert wrapped.temperatures_c == {1: 32.0, 2: 30.0, 3: 29.0}

    duplicate = parse_servo_temperature_line("[SERVO_TEMP] S1:31 S1:32")
    assert duplicate is not None
    assert duplicate.temperatures_c == {1: 32.0}

    assert parse_servo_temperature_line("[SERVO_TEMP] S1:bad S2:30") is None
