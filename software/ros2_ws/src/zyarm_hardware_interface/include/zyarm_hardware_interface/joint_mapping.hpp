#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "zyarm_hardware_interface/shell_protocol.hpp"

namespace zyarm_hardware_interface
{

struct JointMappingConfig
{
  std::array<double, kArmJointCount> arm_hw_offsets_deg{0.0, -180.0, 90.0, 0.0, 0.0, 0.0};
  std::array<double, kArmJointCount> arm_hw_signs{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double claw_travel_m{0.034};
  double claw_command_max{100.0};
};

class JointMapping
{
public:
  JointMapping() = default;
  explicit JointMapping(JointMappingConfig config);

  static const std::vector<std::string> & expected_joint_names();
  static bool has_expected_joint_names(const std::vector<std::string> & joint_names);
  static JointMappingConfig config_from_parameters(
    const std::unordered_map<std::string, std::string> & parameters);

  std::array<double, kJointCount> ros_to_hardware(
    const std::array<double, kJointCount> & ros_positions) const;
  std::array<double, kJointCount> hardware_to_ros(
    const std::array<double, kJointCount> & hardware_positions) const;

  const JointMappingConfig & config() const;

private:
  JointMappingConfig config_;
};

std::array<double, kArmJointCount> parse_six_doubles(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key,
  const std::array<double, kArmJointCount> & fallback);

}  // namespace zyarm_hardware_interface
