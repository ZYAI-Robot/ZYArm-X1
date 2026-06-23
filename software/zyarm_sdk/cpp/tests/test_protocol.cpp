#include <cassert>

#include "zyarm_sdk/protocol.hpp"

int main()
{
  using namespace zyarm_sdk;
  assert(format_command(CommandId::Status) == "[CMD][6]\n");
  JointArray hardware{0, -180, 90, -999.9, 50, 1.25, 100};
  assert(format_joint_io_fast_command(hardware) == "[CMD][36][0 -180 90 -999.900 50 1.250 100]\n");

  auto ack = parse_ack("ACK_COMPLETED: CMD_ID=36, SUCCESS");
  assert(ack.has_value());
  assert(ack->command_id == 36);
  assert(ack->success);

  auto status = parse_status_line("[STATUS] J0:1 J1:-2.5 J2:3 J3:4.25 J4:5 J5:6 CLAW:7.5");
  assert(status.has_value());
  assert(status->values[1] == -2.5);
  assert(!parse_status_line("[STATUS] J0:bad J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7").has_value());

  auto md = parse_master_data_line("[MD][4][10 20 30 40 50 60 70]");
  assert(md.has_value());
  assert(md->frame_id == 4);
  assert(md->values[6] == 70);
  assert(!parse_master_data_line("[MD][4][10 20 30 40 50 60]").has_value());

  auto temps = parse_servo_temperature_line(
    "[SERVO_TEMP] S1:32 S2:30.5 S3:29 S4:28 S5:27 S6:29 S7:29 S8:29 S9:30",
    7);
  assert(temps.has_value());
  assert(temps->sequence == 7);
  assert(temps->temperatures_c.at(1) == 32);
  assert(temps->temperatures_c.at(2) == 30.5);
  assert(temps->temperatures_c.at(9) == 30);

  auto wrapped = parse_servo_temperature_line(
    "ACK_RESPONSE: CMD_ID=6, [SERVO_TEMP] S1:32 S2:30 S3:29");
  assert(wrapped.has_value());
  assert(wrapped->temperatures_c.at(1) == 32);
  assert(wrapped->temperatures_c.at(3) == 29);
  assert(!parse_servo_temperature_line("[SERVO_TEMP] S1:bad S2:30").has_value());
  assert(!parse_servo_temperature_line("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:50").has_value());
  return 0;
}
