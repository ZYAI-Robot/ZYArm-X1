#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <string>

namespace zyarm_sdk
{

inline constexpr std::size_t kArmJointCount = 6;
inline constexpr std::size_t kJointCount = 7;
inline constexpr double kNoChangeSentinel = -999.9;

enum class StateSource
{
  Cache,
  Cmd6Query,
  Cmd36MeasuredSnapshot,
  StatusReport,
  MasterData,
  Unknown
};

using Clock = std::chrono::steady_clock;
using JointArray = std::array<double, kJointCount>;

struct ArmState
{
  JointArray positions{};
  StateSource source{StateSource::Unknown};
  Clock::time_point timestamp{Clock::now()};
  std::uint64_t sequence{0};
  std::string raw_line;

  double age_ms() const;
};

struct CommandResult
{
  bool accepted{false};
  std::string message;
  int command_id{0};
  Clock::time_point dispatched_at{Clock::now()};
};

struct FastIoResult
{
  bool accepted{false};
  std::string message;
  int command_id{36};
  Clock::time_point dispatched_at{Clock::now()};
  std::optional<ArmState> measured_snapshot;
  bool state_waited{false};
};

struct TeleopAction
{
  JointArray positions{};
  StateSource source{StateSource::MasterData};
  Clock::time_point timestamp{Clock::now()};
  std::uint64_t sequence{0};
  std::string raw_line;

  double age_ms() const;
};

struct TeleopStepResult
{
  TeleopAction action;
  FastIoResult command;
  std::optional<ArmState> observation;
};

struct ArmFrameStats
{
  std::uint64_t master_data_received{0};
  std::uint64_t master_data_gap_count{0};
  double master_data_rate_hz{0.0};
  std::uint64_t status_received{0};
  double status_rate_hz{0.0};
};

}  // namespace zyarm_sdk
