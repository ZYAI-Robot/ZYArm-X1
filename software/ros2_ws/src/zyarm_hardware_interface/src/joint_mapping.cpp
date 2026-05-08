#include "zyarm_hardware_interface/joint_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace zyarm_hardware_interface
{

namespace
{
constexpr double kPi = 3.14159265358979323846;

double radians_to_degrees(double radians)
{
  return radians * 180.0 / kPi;
}

double degrees_to_radians(double degrees)
{
  return degrees * kPi / 180.0;
}

double get_double(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key,
  double fallback)
{
  auto iter = parameters.find(key);
  if (iter == parameters.end() || iter->second.empty()) {
    return fallback;
  }
  return std::stod(iter->second);
}

std::vector<double> parse_double_list(const std::string & text)
{
  std::string normalized = text;
  std::replace(normalized.begin(), normalized.end(), ',', ' ');

  std::istringstream stream(normalized);
  std::vector<double> values;
  double value = 0.0;
  while (stream >> value) {
    values.push_back(value);
  }
  return values;
}
}  // namespace

JointMapping::JointMapping(JointMappingConfig config)
: config_(config)
{
}

const std::vector<std::string> & JointMapping::expected_joint_names()
{
  static const std::vector<std::string> names{
    "joint0", "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  return names;
}

bool JointMapping::has_expected_joint_names(const std::vector<std::string> & joint_names)
{
  return joint_names == expected_joint_names();
}

JointMappingConfig JointMapping::config_from_parameters(
  const std::unordered_map<std::string, std::string> & parameters)
{
  JointMappingConfig config;
  config.arm_hw_offsets_deg =
    parse_six_doubles(parameters, "arm_hw_offsets_deg", config.arm_hw_offsets_deg);
  config.arm_hw_signs = parse_six_doubles(parameters, "arm_hw_signs", config.arm_hw_signs);
  config.claw_travel_m = get_double(parameters, "claw_travel_m", config.claw_travel_m);
  config.claw_command_max = get_double(parameters, "claw_command_max", config.claw_command_max);
  if (config.claw_travel_m <= 0.0) {
    throw std::invalid_argument("claw_travel_m must be positive");
  }
  if (config.claw_command_max <= 0.0) {
    throw std::invalid_argument("claw_command_max must be positive");
  }
  return config;
}

std::array<double, kJointCount> JointMapping::ros_to_hardware(
  const std::array<double, kJointCount> & ros_positions) const
{
  std::array<double, kJointCount> hardware{};
  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    hardware[index] = radians_to_degrees(ros_positions[index]) * config_.arm_hw_signs[index] +
      config_.arm_hw_offsets_deg[index];
  }

  const double claw = std::clamp(ros_positions[kArmJointCount], 0.0, config_.claw_travel_m);
  hardware[kArmJointCount] = claw / config_.claw_travel_m * config_.claw_command_max;
  return hardware;
}

std::array<double, kJointCount> JointMapping::hardware_to_ros(
  const std::array<double, kJointCount> & hardware_positions) const
{
  std::array<double, kJointCount> ros{};
  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    ros[index] = degrees_to_radians(
      (hardware_positions[index] - config_.arm_hw_offsets_deg[index]) /
      config_.arm_hw_signs[index]);
  }

  const double claw = std::clamp(hardware_positions[kArmJointCount], 0.0, config_.claw_command_max);
  ros[kArmJointCount] = claw / config_.claw_command_max * config_.claw_travel_m;
  return ros;
}

const JointMappingConfig & JointMapping::config() const
{
  return config_;
}

std::array<double, kArmJointCount> parse_six_doubles(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & key,
  const std::array<double, kArmJointCount> & fallback)
{
  auto iter = parameters.find(key);
  if (iter == parameters.end() || iter->second.empty()) {
    return fallback;
  }

  const auto values = parse_double_list(iter->second);
  if (values.size() != kArmJointCount) {
    throw std::invalid_argument(key + " must contain exactly 6 numeric values");
  }

  std::array<double, kArmJointCount> parsed{};
  std::copy(values.begin(), values.end(), parsed.begin());
  return parsed;
}

}  // namespace zyarm_hardware_interface
