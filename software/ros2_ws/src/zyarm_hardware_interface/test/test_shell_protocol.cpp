#include "zyarm_hardware_interface/shell_protocol.hpp"

#include <array>

#include "gtest/gtest.h"

namespace zyarm_hardware_interface
{

TEST(ShellProtocol, FormatsCmd36WithFirmwareNumberStyle)
{
  const std::array<double, kJointCount> positions{
    0.0, 1.23456, -2.0, -999.9, 4.2, 5.0, 6.7894};

  EXPECT_EQ(
    format_joint_io_fast_command(positions),
    "[CMD][36][0 1.235 -2 -999.900 4.200 5 6.789]\n");
}

TEST(ShellProtocol, ParsesCompleteStatusFrame)
{
  const auto values = parse_status_values(
    "[STATUS] J0:1 J1:-2.5 J2:3 J3:4.25 J4:5 J5:6 CLAW:7.5");

  ASSERT_TRUE(values.has_value());
  EXPECT_DOUBLE_EQ((*values)[0], 1.0);
  EXPECT_DOUBLE_EQ((*values)[1], -2.5);
  EXPECT_DOUBLE_EQ((*values)[2], 3.0);
  EXPECT_DOUBLE_EQ((*values)[3], 4.25);
  EXPECT_DOUBLE_EQ((*values)[4], 5.0);
  EXPECT_DOUBLE_EQ((*values)[5], 6.0);
  EXPECT_DOUBLE_EQ((*values)[6], 7.5);
}

TEST(ShellProtocol, BuildsStatusFrameWithRawLineAndTimestamp)
{
  const auto received_at = std::chrono::steady_clock::now();
  const auto frame = parse_status_frame(
    "prefix [STATUS] J0:1 J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7", received_at);

  ASSERT_TRUE(frame.has_value());
  EXPECT_EQ(frame->received_at, received_at);
  EXPECT_EQ(frame->raw_line, "prefix [STATUS] J0:1 J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7");
  EXPECT_DOUBLE_EQ(frame->hardware_positions[6], 7.0);
}

TEST(ShellProtocol, RejectsMalformedOrIncompleteStatusLines)
{
  EXPECT_FALSE(parse_status_values("hello").has_value());
  EXPECT_FALSE(parse_status_values("[STATUS] J0:1 J1:2 J2:3 J3:4 J4:5 J5:6").has_value());
  EXPECT_FALSE(
    parse_status_values("[STATUS] J0:--1 J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7").has_value());
  EXPECT_FALSE(
    parse_status_values("[STATUS] J0:1.2.3 J1:2 J2:3 J3:4 J4:5 J5:6 CLAW:7").has_value());
}

}  // namespace zyarm_hardware_interface
