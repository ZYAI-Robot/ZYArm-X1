import math

from zyarm_sdk.protocol import (
    CommandId,
    format_command,
    format_joint_io_fast_command,
    parse_ack,
    parse_master_data_line,
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
