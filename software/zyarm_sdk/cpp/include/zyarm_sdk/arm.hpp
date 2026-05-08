#pragma once

#include <memory>
#include <optional>

#include "zyarm_sdk/config.hpp"
#include "zyarm_sdk/mapping.hpp"
#include "zyarm_sdk/safety.hpp"
#include "zyarm_sdk/transport.hpp"

namespace zyarm_sdk
{

class ZyArm
{
public:
  explicit ZyArm(ZyArmConfig config);
  ZyArm(ZyArmConfig config, std::shared_ptr<Transport> transport);

  ZyArm & connect();
  void close();
  bool is_connected() const;

  CommandResult reset(std::chrono::milliseconds timeout = std::chrono::milliseconds(0));
  CommandResult stop(std::chrono::milliseconds timeout = std::chrono::milliseconds(0));
  CommandResult send_command(
    int command_id,
    const std::vector<double> & params = {},
    bool wait_ack = true,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0));
  CommandResult move_ik(double x, double y, double z, double rx = 0.0, double ry = 0.0, double rz = 0.0);
  CommandResult set_gripper(double position, bool sync = false);
  CommandResult play_record(
    int action_id,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0));
  CommandResult set_speed(double speed);
  std::optional<ArmState> get_latest_state(std::optional<double> max_age_ms = std::nullopt) const;
  std::optional<ArmState> query_state(std::chrono::milliseconds timeout);
  FastIoResult fast_io(
    const JointArray & positions,
    const std::array<bool, kJointCount> & apply_mask = all_joints_mask(),
    bool wait_state = false,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(50));

  CommandResult enter_master_slave_mode(MasterSlaveRole role, double frequency_hz = 50.0);
  CommandResult enter_master_mode(double frequency_hz = 50.0);
  CommandResult enter_slave_mode(double frequency_hz = 50.0);
  CommandResult stop_master_mode();
  CommandResult set_master_slave_lpf(double alpha);
  ArmFrameStats get_frame_stats() const;
  void reset_frame_stats();

  std::shared_ptr<Transport> transport() const;
  const JointMapping & mapping() const;

private:
  ArmState state_from_frame(const StatusFrame & frame, StateSource source) const;

  ZyArmConfig config_;
  JointMapping mapping_;
  SafetyController safety_;
  std::shared_ptr<Transport> transport_;
};

}  // namespace zyarm_sdk
