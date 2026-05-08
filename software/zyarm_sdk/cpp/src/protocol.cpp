#include "zyarm_sdk/protocol.hpp"

#include <cmath>
#include <iomanip>
#include <regex>
#include <sstream>

namespace zyarm_sdk
{

namespace
{
const std::regex kAckRegex(R"(ACK_COMPLETED:\s*CMD_ID=(\d+),\s*(SUCCESS|ERROR))");
const std::regex kStatusRegex(
  R"(\[STATUS\]\s*J0:([-\d.]+)\s*J1:([-\d.]+)\s*J2:([-\d.]+)\s*J3:([-\d.]+)\s*J4:([-\d.]+)\s*J5:([-\d.]+)\s*CLAW:([-\d.]+))");
const std::regex kMasterDataRegex(
  R"(\[MD\]\[(\d+)\]\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\])");

std::optional<JointArray> parse_values(const std::smatch & match, std::size_t first)
{
  JointArray values{};
  try {
    for (std::size_t index = 0; index < kJointCount; ++index) {
      const auto field = match[first + index].str();
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
}  // namespace

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

std::string format_command(CommandId command_id, const std::vector<double> & params)
{
  return format_command(static_cast<int>(command_id), params);
}

std::string format_joint_io_fast_command(const JointArray & hardware_positions)
{
  return format_command(
    CommandId::JointIoFast,
    std::vector<double>(hardware_positions.begin(), hardware_positions.end()));
}

std::optional<AckFrame> parse_ack(const std::string & line)
{
  std::smatch match;
  if (!std::regex_search(line, match, kAckRegex) || match.size() != 3) {
    return std::nullopt;
  }
  AckFrame frame;
  frame.command_id = std::stoi(match[1].str());
  frame.success = match[2].str() == "SUCCESS";
  frame.raw_line = line;
  return frame;
}

std::optional<StatusFrame> parse_status_line(
  const std::string & line,
  std::uint64_t sequence,
  Clock::time_point received_at)
{
  std::smatch match;
  if (!std::regex_search(line, match, kStatusRegex) || match.size() != kJointCount + 1) {
    return std::nullopt;
  }
  auto values = parse_values(match, 1);
  if (!values.has_value()) {
    return std::nullopt;
  }
  StatusFrame frame;
  frame.values = *values;
  frame.received_at = received_at;
  frame.sequence = sequence;
  frame.raw_line = line;
  return frame;
}

std::optional<MasterDataFrame> parse_master_data_line(
  const std::string & line,
  std::uint64_t sequence,
  Clock::time_point received_at)
{
  std::smatch match;
  if (!std::regex_search(line, match, kMasterDataRegex) || match.size() != kJointCount + 2) {
    return std::nullopt;
  }
  auto values = parse_values(match, 2);
  if (!values.has_value()) {
    return std::nullopt;
  }
  MasterDataFrame frame;
  frame.frame_id = std::stoi(match[1].str());
  frame.values = *values;
  frame.received_at = received_at;
  frame.sequence = sequence;
  frame.raw_line = line;
  return frame;
}

}  // namespace zyarm_sdk
