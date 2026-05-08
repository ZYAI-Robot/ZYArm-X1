#include "zyarm_hardware_interface/joint_mapping.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"

namespace zyarm_hardware_interface
{

namespace
{
constexpr double kTolerance = 1e-9;
constexpr double kPi = 3.14159265358979323846;
}

TEST(JointMapping, RequiresJoint0ThroughJoint6Only)
{
  EXPECT_TRUE(JointMapping::has_expected_joint_names(
    {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"}));
  EXPECT_FALSE(JointMapping::has_expected_joint_names(
    {"joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"}));
  EXPECT_FALSE(JointMapping::has_expected_joint_names(
    {"joint0", "joint1", "joint2", "joint3", "joint4", "joint6", "joint5"}));
}

TEST(JointMapping, AppliesDefaultArmOffsetsAndClawScale)
{
  JointMapping mapping;
  std::array<double, kJointCount> ros_positions{};
  ros_positions.fill(0.0);
  ros_positions[6] = 0.017;

  const auto hardware = mapping.ros_to_hardware(ros_positions);
  EXPECT_NEAR(hardware[0], 0.0, kTolerance);
  EXPECT_NEAR(hardware[1], -180.0, kTolerance);
  EXPECT_NEAR(hardware[2], 90.0, kTolerance);
  EXPECT_NEAR(hardware[3], 0.0, kTolerance);
  EXPECT_NEAR(hardware[4], 0.0, kTolerance);
  EXPECT_NEAR(hardware[5], 0.0, kTolerance);
  EXPECT_NEAR(hardware[6], 50.0, kTolerance);

  ros_positions[6] = -1.0;
  EXPECT_NEAR(mapping.ros_to_hardware(ros_positions)[6], 0.0, kTolerance);
  ros_positions[6] = 1.0;
  EXPECT_NEAR(mapping.ros_to_hardware(ros_positions)[6], 100.0, kTolerance);
}

TEST(JointMapping, AppliesCustomSignsOffsetsAndInverseMapping)
{
  const std::unordered_map<std::string, std::string> params{
    {"arm_hw_offsets_deg", "10,20,30,40,50,60"},
    {"arm_hw_signs", "1 -1 1 -1 1 -1"},
    {"claw_travel_m", "0.040"},
    {"claw_command_max", "200"},
  };
  JointMapping mapping(JointMapping::config_from_parameters(params));

  const std::array<double, kJointCount> ros_positions{
    kPi / 2.0, kPi / 2.0, -kPi / 2.0, -kPi / 2.0, 0.0, kPi, 0.020};

  const auto hardware = mapping.ros_to_hardware(ros_positions);
  EXPECT_NEAR(hardware[0], 100.0, kTolerance);
  EXPECT_NEAR(hardware[1], -70.0, kTolerance);
  EXPECT_NEAR(hardware[2], -60.0, kTolerance);
  EXPECT_NEAR(hardware[3], 130.0, kTolerance);
  EXPECT_NEAR(hardware[4], 50.0, kTolerance);
  EXPECT_NEAR(hardware[5], -120.0, kTolerance);
  EXPECT_NEAR(hardware[6], 100.0, kTolerance);

  const auto round_trip = mapping.hardware_to_ros(hardware);
  for (std::size_t index = 0; index < kJointCount; ++index) {
    EXPECT_NEAR(round_trip[index], ros_positions[index], kTolerance);
  }
}

TEST(JointMapping, RejectsInvalidMappingParameters)
{
  EXPECT_THROW(
    JointMapping::config_from_parameters({{"arm_hw_offsets_deg", "1 2 3"}}),
    std::invalid_argument);
  EXPECT_THROW(
    JointMapping::config_from_parameters({{"arm_hw_signs", "1 1 1 1 1 x"}}),
    std::invalid_argument);
  EXPECT_THROW(
    JointMapping::config_from_parameters({{"claw_travel_m", "0"}}),
    std::invalid_argument);
  EXPECT_THROW(
    JointMapping::config_from_parameters({{"claw_command_max", "-1"}}),
    std::invalid_argument);
}

}  // namespace zyarm_hardware_interface
