#pragma once

#include <array>
#include <optional>

#include "zyarm_sdk/types.hpp"

namespace zyarm_sdk
{

struct RetargetConfig
{
  JointArray signs{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  JointArray offsets{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  JointArray deadband{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::optional<JointArray> min_positions;
  std::optional<JointArray> max_positions;
  std::optional<JointArray> max_delta;
  std::array<bool, kJointCount> apply_mask{true, true, true, true, true, true, true};
};

class Retargeter
{
public:
  Retargeter();
  explicit Retargeter(RetargetConfig config);

  JointArray apply(const TeleopAction & action);
  std::array<bool, kJointCount> apply_mask() const;

private:
  RetargetConfig config_;
  std::optional<JointArray> last_;
};

}  // namespace zyarm_sdk
