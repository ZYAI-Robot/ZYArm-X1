#include "zyarm_sdk/mapping.hpp"

#include <algorithm>
#include <cmath>

namespace zyarm_sdk
{

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

JointMapping::JointMapping() = default;

JointMapping::JointMapping(MappingConfig config) : config_(config) {}

JointArray JointMapping::public_to_hardware(
  const JointArray & positions,
  const std::array<bool, kJointCount> & apply_mask) const
{
  JointArray hardware{};
  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    hardware[index] = apply_mask[index] ?
      positions[index] * 180.0 / kPi * config_.arm_signs[index] +
        config_.arm_offsets_deg[index] :
      kNoChangeSentinel;
  }
  if (apply_mask[kArmJointCount]) {
    const double gripper = std::clamp(positions[kArmJointCount], 0.0, 1.0);
    hardware[kArmJointCount] = gripper * config_.gripper_command_max;
  } else {
    hardware[kArmJointCount] = kNoChangeSentinel;
  }
  return hardware;
}

JointArray JointMapping::hardware_to_public(const JointArray & positions) const
{
  JointArray public_positions{};
  for (std::size_t index = 0; index < kArmJointCount; ++index) {
    public_positions[index] =
      (positions[index] - config_.arm_offsets_deg[index]) / config_.arm_signs[index] * kPi / 180.0;
  }
  const double gripper = std::clamp(positions[kArmJointCount], 0.0, config_.gripper_command_max);
  public_positions[kArmJointCount] = gripper / config_.gripper_command_max;
  return public_positions;
}

}  // namespace zyarm_sdk
