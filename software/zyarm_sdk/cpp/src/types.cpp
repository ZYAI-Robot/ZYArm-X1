#include "zyarm_sdk/types.hpp"

namespace zyarm_sdk
{

double ArmState::age_ms() const
{
  return std::chrono::duration<double, std::milli>(Clock::now() - timestamp).count();
}

double TeleopAction::age_ms() const
{
  return std::chrono::duration<double, std::milli>(Clock::now() - timestamp).count();
}

}  // namespace zyarm_sdk
