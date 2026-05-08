#pragma once

#include <optional>
#include <string>
#include <vector>

#include "zyarm_sdk/types.hpp"

namespace zyarm_sdk
{

enum class CommandId
{
  IkInverse = 0,
  Reset = 1,
  Stop = 2,
  JointSync = 3,
  Status = 6,
  SetClaw = 8,
  SetJointSpeed = 11,
  RecordPlayer = 14,
  StatusReport = 17,
  RemoteMode = 24,
  RemoteReset = 25,
  Remote = 26,
  MasterSlave = 32,
  MasterSlaveStop = 33,
  MasterSlaveSetLpf = 34,
  JointIoFast = 36
};

enum class MasterSlaveRole
{
  Master = 1,
  Slave = 2
};

struct AckFrame
{
  int command_id{0};
  bool success{false};
  std::string raw_line;
};

struct StatusFrame
{
  JointArray values{};
  Clock::time_point received_at{Clock::now()};
  std::uint64_t sequence{0};
  std::string raw_line;
};

struct MasterDataFrame
{
  int frame_id{0};
  JointArray values{};
  Clock::time_point received_at{Clock::now()};
  std::uint64_t sequence{0};
  std::string raw_line;
};

std::string format_number(double value);
std::string format_command(int command_id, const std::vector<double> & params = {});
std::string format_command(CommandId command_id, const std::vector<double> & params = {});
std::string format_joint_io_fast_command(const JointArray & hardware_positions);

std::optional<AckFrame> parse_ack(const std::string & line);
std::optional<StatusFrame> parse_status_line(
  const std::string & line,
  std::uint64_t sequence = 0,
  Clock::time_point received_at = Clock::now());
std::optional<MasterDataFrame> parse_master_data_line(
  const std::string & line,
  std::uint64_t sequence = 0,
  Clock::time_point received_at = Clock::now());

}  // namespace zyarm_sdk
