#pragma once

#include <array>

#include "zyarm_sdk/config.hpp"
#include "zyarm_sdk/types.hpp"

namespace zyarm_sdk
{

class JointMapping
{
public:
  JointMapping();
  explicit JointMapping(MappingConfig config);

  JointArray public_to_hardware(
    const JointArray & positions,
    const std::array<bool, kJointCount> & apply_mask) const;
  JointArray hardware_to_public(const JointArray & positions) const;

private:
  MappingConfig config_;
};

}  // namespace zyarm_sdk
