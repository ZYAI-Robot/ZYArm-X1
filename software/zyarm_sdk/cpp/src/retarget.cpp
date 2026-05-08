#include "zyarm_sdk/retarget.hpp"

#include <algorithm>
#include <cmath>

namespace zyarm_sdk
{

Retargeter::Retargeter() = default;

Retargeter::Retargeter(RetargetConfig config) : config_(config) {}

JointArray Retargeter::apply(const TeleopAction & action)
{
  JointArray out{};
  for (std::size_t index = 0; index < kJointCount; ++index) {
    double value = action.positions[index] * config_.signs[index] + config_.offsets[index];
    if (last_.has_value() && std::abs(value - (*last_)[index]) < config_.deadband[index]) {
      value = (*last_)[index];
    }
    if (last_.has_value() && config_.max_delta.has_value()) {
      const double delta = value - (*last_)[index];
      const double step = (*config_.max_delta)[index];
      if (delta > step) {
        value = (*last_)[index] + step;
      } else if (delta < -step) {
        value = (*last_)[index] - step;
      }
    }
    if (config_.min_positions.has_value()) {
      value = std::max(value, (*config_.min_positions)[index]);
    }
    if (config_.max_positions.has_value()) {
      value = std::min(value, (*config_.max_positions)[index]);
    }
    out[index] = value;
  }
  last_ = out;
  return out;
}

std::array<bool, kJointCount> Retargeter::apply_mask() const
{
  return config_.apply_mask;
}

}  // namespace zyarm_sdk
