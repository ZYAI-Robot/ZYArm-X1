#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <thread>

#include "zyarm_sdk/arm.hpp"
#include "zyarm_sdk/retarget.hpp"

namespace zyarm_sdk
{

class ZyArmLeader
{
public:
  ZyArmLeader(std::shared_ptr<ZyArm> arm, TeleopConfig config = {});
  void start();
  void stop();
  std::optional<TeleopAction> get_action(bool wait = false);
  TeleopAction action_from_frame(const MasterDataFrame & frame) const;

private:
  std::shared_ptr<ZyArm> arm_;
  TeleopConfig config_;
};

class ZyArmFollower
{
public:
  ZyArmFollower(
    std::shared_ptr<ZyArm> arm,
    RetargetConfig retarget = {},
    TeleopConfig config = {});
  TeleopStepResult send_action(const TeleopAction & action, bool wait_state = false);
  std::optional<ArmState> get_observation() const;

private:
  std::shared_ptr<ZyArm> arm_;
  TeleopConfig config_;
  Retargeter retargeter_;
};

class ZyArmTeleopPair
{
public:
  ZyArmTeleopPair(
    std::shared_ptr<ZyArm> leader,
    std::shared_ptr<ZyArm> follower,
    TeleopConfig config = {},
    RetargetConfig retarget = {});
  ~ZyArmTeleopPair();

  ZyArmTeleopPair & connect();
  void start_step_mode();
  std::optional<TeleopStepResult> step(bool wait_action = false, bool wait_state = false);
  void start_auto_follow();
  void stop();
  void close();

private:
  void auto_follow_loop(std::uint64_t last_sequence);
  void start_follower_slave_mode();
  void stop_follower_slave_mode();

  TeleopConfig config_;
  std::shared_ptr<ZyArm> leader_arm_;
  std::shared_ptr<ZyArm> follower_arm_;
  ZyArmLeader leader_;
  ZyArmFollower follower_;
  std::atomic<bool> stop_follow_{false};
  std::thread follow_thread_;
  bool follower_slave_started_{false};
};

}  // namespace zyarm_sdk
