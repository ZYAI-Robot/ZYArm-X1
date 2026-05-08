#include "zyarm_sdk/teleop.hpp"

#include <chrono>
#include <thread>
#include <utility>

#include "zyarm_sdk/errors.hpp"

namespace zyarm_sdk
{

ZyArmLeader::ZyArmLeader(std::shared_ptr<ZyArm> arm, TeleopConfig config)
: arm_(std::move(arm)), config_(config)
{
}

void ZyArmLeader::start()
{
  arm_->enter_master_mode(config_.leader_hz);
}

void ZyArmLeader::stop()
{
  arm_->stop_master_mode();
}

std::optional<TeleopAction> ZyArmLeader::get_action(bool wait)
{
  std::optional<MasterDataFrame> frame;
  if (wait) {
    const auto before = arm_->transport()->master_data_sequence();
    frame = arm_->transport()->wait_for_master_data_after(before, config_.wait_timeout);
  } else {
    frame = arm_->transport()->latest_master_data();
  }
  if (!frame.has_value()) {
    return std::nullopt;
  }
  return action_from_frame(*frame);
}

TeleopAction ZyArmLeader::action_from_frame(const MasterDataFrame & frame) const
{
  TeleopAction action{
    arm_->mapping().hardware_to_public(frame.values),
    StateSource::MasterData,
    frame.received_at,
    frame.sequence,
    frame.raw_line};
  if (action.age_ms() > config_.action_max_age_ms) {
    throw StaleStateError("leader action is stale");
  }
  return action;
}

ZyArmFollower::ZyArmFollower(
  std::shared_ptr<ZyArm> arm,
  RetargetConfig retarget,
  TeleopConfig config)
: arm_(std::move(arm)), config_(config), retargeter_(retarget)
{
}

TeleopStepResult ZyArmFollower::send_action(const TeleopAction & action, bool wait_state)
{
  const auto positions = retargeter_.apply(action);
  auto command = arm_->fast_io(positions, retargeter_.apply_mask(), wait_state, config_.wait_timeout);
  auto observation = arm_->get_latest_state(config_.state_max_age_ms);
  return TeleopStepResult{action, command, observation};
}

std::optional<ArmState> ZyArmFollower::get_observation() const
{
  return arm_->get_latest_state(config_.state_max_age_ms);
}

ZyArmTeleopPair::ZyArmTeleopPair(
  std::shared_ptr<ZyArm> leader,
  std::shared_ptr<ZyArm> follower,
  TeleopConfig config,
  RetargetConfig retarget)
: config_(config),
  leader_arm_(leader),
  follower_arm_(follower),
  leader_(leader, config),
  follower_(follower, retarget, config)
{
}

ZyArmTeleopPair::~ZyArmTeleopPair()
{
  stop();
}

ZyArmTeleopPair & ZyArmTeleopPair::connect()
{
  leader_arm_->connect();
  follower_arm_->connect();
  return *this;
}

void ZyArmTeleopPair::start_step_mode()
{
  try {
    start_follower_slave_mode();
    leader_.start();
  } catch (...) {
    stop_follower_slave_mode();
    throw;
  }
}

std::optional<TeleopStepResult> ZyArmTeleopPair::step(bool wait_action, bool wait_state)
{
  auto action = leader_.get_action(wait_action);
  if (!action.has_value()) {
    return std::nullopt;
  }
  return follower_.send_action(*action, wait_state);
}

void ZyArmTeleopPair::start_auto_follow()
{
  if (follow_thread_.joinable()) {
    return;
  }
  stop_follow_ = false;
  start_step_mode();
  const auto last_sequence = leader_arm_->transport()->master_data_sequence();
  follow_thread_ = std::thread(&ZyArmTeleopPair::auto_follow_loop, this, last_sequence);
}

void ZyArmTeleopPair::stop()
{
  stop_follow_ = true;
  if (follow_thread_.joinable()) {
    follow_thread_.join();
  }
  leader_.stop();
  stop_follower_slave_mode();
}

void ZyArmTeleopPair::close()
{
  stop();
  leader_arm_->close();
  follower_arm_->close();
}

void ZyArmTeleopPair::auto_follow_loop(std::uint64_t last_sequence)
{
  while (!stop_follow_) {
    auto frame = leader_arm_->transport()->wait_for_master_data_after(
      last_sequence,
      config_.wait_timeout);
    if (!frame.has_value()) {
      continue;
    }
    last_sequence = frame->sequence;

    TeleopAction action;
    try {
      action = leader_.action_from_frame(*frame);
      follower_.send_action(action, false);
    } catch (...) {
      continue;
    }
  }
}

void ZyArmTeleopPair::start_follower_slave_mode()
{
  if (follower_slave_started_) {
    return;
  }
  auto result = follower_arm_->set_master_slave_lpf(config_.slave_filter_lpf_alpha);
  if (!result.accepted) {
    throw ZyArmError("follower slave filter LPF configuration failed");
  }
  result = follower_arm_->enter_slave_mode();
  if (!result.accepted) {
    throw ZyArmError("follower slave mode start failed");
  }
  follower_slave_started_ = true;
}

void ZyArmTeleopPair::stop_follower_slave_mode()
{
  if (!follower_slave_started_) {
    return;
  }
  follower_arm_->stop_master_mode();
  follower_slave_started_ = false;
}

}  // namespace zyarm_sdk
