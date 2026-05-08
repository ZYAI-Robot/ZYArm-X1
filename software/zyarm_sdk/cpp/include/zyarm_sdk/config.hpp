#pragma once

#include <array>
#include <chrono>
#include <optional>
#include <string>

#include "zyarm_sdk/types.hpp"

namespace zyarm_sdk
{

struct MappingConfig
{
  std::array<double, kArmJointCount> arm_offsets_deg{0.0, -180.0, 90.0, 0.0, 0.0, 0.0};
  std::array<double, kArmJointCount> arm_signs{1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
  double gripper_command_max{100.0};
};

struct SafetyConfig
{
  std::optional<JointArray> min_positions;
  std::optional<JointArray> max_positions;
  std::optional<JointArray> max_delta;
};

struct ZyArmConfig
{
  std::string port;
  int baudrate{230400};
  std::chrono::milliseconds timeout{20};
  std::chrono::milliseconds write_timeout{50};
  // 普通 ACK 通常来自配置/模式切换命令，应快速暴露串口、波特率或协议错误。
  std::chrono::milliseconds ack_timeout{1000};
  // 动作 ACK 表示固件报告动作执行完成，reset/move_ik/同步夹爪可能需要更久。
  std::chrono::milliseconds action_timeout{10000};
  // 录制动作最长可达 3 分钟，回放完成 ACK 需要预留少量收尾和串口调度余量。
  std::chrono::milliseconds play_record_timeout{190000};
  bool reset_rts_dtr{false};
  std::chrono::milliseconds reset_quiet{0};
  MappingConfig mapping;
  SafetyConfig safety;
};

struct TeleopConfig
{
  double leader_hz{50.0};
  double action_max_age_ms{100.0};
  std::optional<double> state_max_age_ms;
  std::chrono::milliseconds wait_timeout{50};
  double slave_filter_lpf_alpha{0.15};
  bool verbose{false};
};

}  // namespace zyarm_sdk
