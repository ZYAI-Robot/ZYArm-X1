#include "zyarm_sdk/arm.hpp"

#include <algorithm>
#include <utility>

#include "zyarm_sdk/errors.hpp"
#include "zyarm_sdk/protocol.hpp"

namespace zyarm_sdk
{

ZyArm::ZyArm(ZyArmConfig config)
: ZyArm(config, std::make_shared<SerialTransport>(config))
{
}

ZyArm::ZyArm(ZyArmConfig config, std::shared_ptr<Transport> transport)
: config_(std::move(config)),
  mapping_(config_.mapping),
  safety_(config_.safety),
  transport_(std::move(transport))
{
}

ZyArm & ZyArm::connect()
{
  transport_->connect();
  return *this;
}

void ZyArm::close()
{
  transport_->close();
}

bool ZyArm::is_connected() const
{
  return transport_->is_connected();
}

CommandResult ZyArm::reset(std::chrono::milliseconds timeout)
{
  return send_command(
    static_cast<int>(CommandId::Reset),
    {},
    true,
    timeout.count() > 0 ? timeout : config_.action_timeout);
}

CommandResult ZyArm::stop(std::chrono::milliseconds timeout)
{
  return send_command(static_cast<int>(CommandId::Stop), {}, true, timeout);
}

CommandResult ZyArm::send_command(
  int command_id,
  const std::vector<double> & params,
  bool wait_ack,
  std::chrono::milliseconds timeout)
{
  const auto dispatched_at = Clock::now();
  const bool ok = transport_->send_command(
    command_id,
    params,
    wait_ack,
    timeout.count() > 0 ? timeout : config_.ack_timeout);
  return CommandResult{
    ok,
    ok ? "command dispatched" : "ACK timeout or error",
    command_id,
    dispatched_at};
}

CommandResult ZyArm::move_ik(double x, double y, double z, double rx, double ry, double rz)
{
  // 动作类 ACK 代表固件执行完成，不只是命令写入成功，因此使用更长的默认等待时间。
  return send_command(
    static_cast<int>(CommandId::IkInverse),
    {x, y, z, rx, ry, rz},
    true,
    config_.action_timeout);
}

CommandResult ZyArm::set_gripper(double position, bool sync)
{
  const double normalized = std::clamp(position, 0.0, 1.0);
  const double hardware = normalized * config_.mapping.gripper_command_max;
  return send_command(
    static_cast<int>(CommandId::SetClaw),
    {hardware, sync ? 1.0 : 0.0},
    sync,
    sync ? config_.action_timeout : std::chrono::milliseconds(0));
}

CommandResult ZyArm::play_record(int action_id, std::chrono::milliseconds timeout)
{
  return send_command(
    static_cast<int>(CommandId::RecordPlayer),
    {static_cast<double>(action_id)},
    true,
    timeout.count() > 0 ? timeout : config_.play_record_timeout);
}

CommandResult ZyArm::set_speed(double speed)
{
  return send_command(static_cast<int>(CommandId::SetJointSpeed), {speed}, false);
}

std::optional<ArmState> ZyArm::get_latest_state(std::optional<double> max_age_ms) const
{
  auto frame = transport_->latest_status();
  if (!frame.has_value()) {
    return std::nullopt;
  }
  auto state = state_from_frame(*frame, StateSource::Cache);
  if (max_age_ms.has_value() && state.age_ms() > *max_age_ms) {
    throw StaleStateError("cached state is stale");
  }
  return state;
}

std::optional<ArmState> ZyArm::query_state(std::chrono::milliseconds timeout)
{
  const auto before = transport_->status_sequence();
  transport_->send_command(static_cast<int>(CommandId::Status), {}, false);
  auto frame = transport_->wait_for_status_after(before, timeout);
  if (!frame.has_value()) {
    return std::nullopt;
  }
  return state_from_frame(*frame, StateSource::Cmd6Query);
}

FastIoResult ZyArm::fast_io(
  const JointArray & positions,
  const std::array<bool, kJointCount> & apply_mask,
  bool wait_state,
  std::chrono::milliseconds timeout)
{
  const auto safe = safety_.sanitize_positions(positions);
  const auto hardware = mapping_.public_to_hardware(safe, apply_mask);
  const auto before = transport_->status_sequence();
  const auto dispatched_at = Clock::now();
  transport_->send_command(
    static_cast<int>(CommandId::JointIoFast),
    std::vector<double>(hardware.begin(), hardware.end()),
    false);
  std::optional<ArmState> measured;
  if (wait_state) {
    auto frame = transport_->wait_for_status_after(before, timeout);
    if (frame.has_value()) {
      measured = state_from_frame(*frame, StateSource::Cmd36MeasuredSnapshot);
    }
  }
  return FastIoResult{
    true,
    "CMD36 dispatched",
    static_cast<int>(CommandId::JointIoFast),
    dispatched_at,
    measured,
    wait_state};
}

CommandResult ZyArm::enter_master_slave_mode(MasterSlaveRole role, double frequency_hz)
{
  return send_command(
    static_cast<int>(CommandId::MasterSlave),
    {static_cast<double>(static_cast<int>(role)), frequency_hz},
    true,
    config_.ack_timeout);
}

CommandResult ZyArm::enter_master_mode(double frequency_hz)
{
  return enter_master_slave_mode(MasterSlaveRole::Master, frequency_hz);
}

CommandResult ZyArm::enter_slave_mode(double frequency_hz)
{
  return enter_master_slave_mode(MasterSlaveRole::Slave, frequency_hz);
}

CommandResult ZyArm::stop_master_mode()
{
  return send_command(static_cast<int>(CommandId::MasterSlaveStop), {}, true, config_.ack_timeout);
}

CommandResult ZyArm::set_master_slave_lpf(double alpha)
{
  return send_command(
    static_cast<int>(CommandId::MasterSlaveSetLpf),
    {alpha},
    true,
    config_.ack_timeout);
}

ArmFrameStats ZyArm::get_frame_stats() const
{
  return transport_->get_frame_stats();
}

void ZyArm::reset_frame_stats()
{
  transport_->reset_frame_stats();
}

std::shared_ptr<Transport> ZyArm::transport() const
{
  return transport_;
}

const JointMapping & ZyArm::mapping() const
{
  return mapping_;
}

ArmState ZyArm::state_from_frame(const StatusFrame & frame, StateSource source) const
{
  return ArmState{
    mapping_.hardware_to_public(frame.values),
    source,
    frame.received_at,
    frame.sequence,
    frame.raw_line};
}

}  // namespace zyarm_sdk
