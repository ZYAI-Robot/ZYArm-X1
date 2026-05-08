#pragma once

#include <array>

#include "zyarm_sdk/config.hpp"
#include "zyarm_sdk/types.hpp"

namespace zyarm_sdk
{

std::array<bool, kJointCount> all_joints_mask();

class SafetyController
{
public:
  SafetyController();
  explicit SafetyController(SafetyConfig config);

  JointArray sanitize_positions(const JointArray & positions);
  void assert_fresh(double age_ms, double max_age_ms) const;

private:
  SafetyConfig config_;
  std::optional<JointArray> last_positions_;
};

}  // namespace zyarm_sdk
