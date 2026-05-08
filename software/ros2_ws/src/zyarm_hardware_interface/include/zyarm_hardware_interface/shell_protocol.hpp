#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <string>
#include <vector>

namespace zyarm_hardware_interface
{

inline constexpr std::size_t kJointCount = 7;
inline constexpr std::size_t kArmJointCount = 6;
inline constexpr int kJointIoFastCommandId = 36;
inline constexpr double kNoChangeSentinel = -999.9;

struct StatusFrame
{
  std::array<double, kJointCount> hardware_positions{};
  std::chrono::steady_clock::time_point received_at{};
  std::string raw_line;
};

std::string format_command(int command_id, const std::vector<double> & params);
std::string format_joint_io_fast_command(const std::array<double, kJointCount> & hardware_positions);
std::optional<std::array<double, kJointCount>> parse_status_values(const std::string & line);
std::optional<StatusFrame> parse_status_frame(
  const std::string & line,
  std::chrono::steady_clock::time_point received_at = std::chrono::steady_clock::now());

}  // namespace zyarm_hardware_interface
