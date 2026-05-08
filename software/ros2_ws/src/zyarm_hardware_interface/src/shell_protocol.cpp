#include "zyarm_hardware_interface/shell_protocol.hpp"

#include <cmath>
#include <iomanip>
#include <regex>
#include <sstream>

namespace zyarm_hardware_interface
{

namespace
{
const std::regex kStatusRegex(
  R"(\[STATUS\]\s*J0:([-\d.]+)\s*J1:([-\d.]+)\s*J2:([-\d.]+)\s*J3:([-\d.]+)\s*J4:([-\d.]+)\s*J5:([-\d.]+)\s*CLAW:([-\d.]+))");

std::string format_number(double value)
{
  const double rounded = std::round(value);
  if (std::abs(value - rounded) < 1e-9) {
    return std::to_string(static_cast<long long>(rounded));
  }

  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}
}  // namespace

std::string format_command(int command_id, const std::vector<double> & params)
{
  std::ostringstream stream;
  stream << "[CMD][" << command_id << "]";
  if (!params.empty()) {
    stream << "[";
    for (std::size_t index = 0; index < params.size(); ++index) {
      if (index > 0) {
        stream << " ";
      }
      stream << format_number(params[index]);
    }
    stream << "]";
  }
  stream << "\n";
  return stream.str();
}

std::string format_joint_io_fast_command(const std::array<double, kJointCount> & hardware_positions)
{
  return format_command(
    kJointIoFastCommandId,
    std::vector<double>(hardware_positions.begin(), hardware_positions.end()));
}

std::optional<std::array<double, kJointCount>> parse_status_values(const std::string & line)
{
  std::smatch match;
  if (!std::regex_search(line, match, kStatusRegex) || match.size() != kJointCount + 1) {
    return std::nullopt;
  }

  std::array<double, kJointCount> values{};
  try {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const auto field = match[index + 1].str();
      std::size_t parsed = 0;
      values[index] = std::stod(field, &parsed);
      if (parsed != field.size()) {
        return std::nullopt;
      }
    }
  } catch (const std::exception &) {
    return std::nullopt;
  }
  return values;
}

std::optional<StatusFrame> parse_status_frame(
  const std::string & line, std::chrono::steady_clock::time_point received_at)
{
  auto values = parse_status_values(line);
  if (!values.has_value()) {
    return std::nullopt;
  }
  StatusFrame frame;
  frame.hardware_positions = *values;
  frame.received_at = received_at;
  frame.raw_line = line;
  return frame;
}

}  // namespace zyarm_hardware_interface
