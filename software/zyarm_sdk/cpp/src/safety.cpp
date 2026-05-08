#include "zyarm_sdk/safety.hpp"

#include <algorithm>

#include "zyarm_sdk/errors.hpp"

namespace zyarm_sdk
{

std::array<bool, kJointCount> all_joints_mask()
{
  return {true, true, true, true, true, true, true};
}

SafetyController::SafetyController() = default;

SafetyController::SafetyController(SafetyConfig config) : config_(config) {}

JointArray SafetyController::sanitize_positions(const JointArray & positions)
{
  JointArray values = positions;
  if (config_.min_positions.has_value()) {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      values[index] = std::max(values[index], (*config_.min_positions)[index]);
    }
  }
  if (config_.max_positions.has_value()) {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      values[index] = std::min(values[index], (*config_.max_positions)[index]);
    }
  }
  if (config_.max_delta.has_value() && last_positions_.has_value()) {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const double delta = values[index] - (*last_positions_)[index];
      const double step = (*config_.max_delta)[index];
      if (delta > step) {
        values[index] = (*last_positions_)[index] + step;
      } else if (delta < -step) {
        values[index] = (*last_positions_)[index] - step;
      }
    }
  }
  last_positions_ = values;
  return values;
}

void SafetyController::assert_fresh(double age_ms, double max_age_ms) const
{
  if (age_ms > max_age_ms) {
    throw StaleStateError("cached state/action is stale");
  }
}

}  // namespace zyarm_sdk
