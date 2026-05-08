#include <cassert>
#include <chrono>
#include <cmath>
#include <memory>

#include "fake_transport.hpp"
#include "zyarm_sdk/arm.hpp"
#include "zyarm_sdk/protocol.hpp"

int main()
{
  using namespace zyarm_sdk;
  ZyArmConfig config;
  config.port = "fake";
  assert(config.baudrate == 230400);
  config.ack_timeout = std::chrono::milliseconds(100);
  config.action_timeout = std::chrono::milliseconds(2500);
  config.play_record_timeout = std::chrono::milliseconds(181000);
  auto fake = std::make_shared<FakeTransport>();
  ZyArm arm(config, fake);
  arm.connect();

  arm.reset();
  assert(fake->command_ids.back() == static_cast<int>(CommandId::Reset));
  assert(fake->wait_acks.back());
  assert(fake->timeouts.back() == config.action_timeout);

  arm.move_ik(200, 0, 100);
  assert(fake->command_ids.back() == static_cast<int>(CommandId::IkInverse));
  assert(fake->wait_acks.back());
  assert(fake->timeouts.back() == config.action_timeout);

  arm.set_gripper(1.0, true);
  assert(fake->command_ids.back() == static_cast<int>(CommandId::SetClaw));
  assert(fake->wait_acks.back());
  assert(fake->timeouts.back() == config.action_timeout);

  arm.play_record(1);
  assert(fake->command_ids.back() == static_cast<int>(CommandId::RecordPlayer));
  assert(fake->wait_acks.back());
  assert(fake->timeouts.back() == config.play_record_timeout);

  arm.enter_master_mode();
  assert(fake->command_ids.back() == static_cast<int>(CommandId::MasterSlave));
  assert(fake->written_lines.back() == "[CMD][32][1 50]\n");
  assert(fake->wait_acks.back());
  assert(fake->timeouts.back() == config.ack_timeout);

  arm.enter_slave_mode();
  assert(fake->command_ids.back() == static_cast<int>(CommandId::MasterSlave));
  assert(fake->written_lines.back() == "[CMD][32][2 50]\n");
  assert(fake->wait_acks.back());
  assert(fake->timeouts.back() == config.ack_timeout);

  arm.set_master_slave_lpf(0.15);
  assert(fake->command_ids.back() == static_cast<int>(CommandId::MasterSlaveSetLpf));
  assert(fake->wait_acks.back());
  assert(fake->timeouts.back() == config.ack_timeout);

  JointArray target{0, 0, 0, 0, 0, 0, 0.5};
  auto result = arm.fast_io(target);
  assert(result.accepted);
  assert(!result.measured_snapshot.has_value());
  assert(fake->written_lines.back().find("[CMD][36]") == 0);

  fake->feed_line("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:50");
  auto latest = arm.get_latest_state();
  assert(latest.has_value());
  assert(latest->source == StateSource::Cache);
  assert(std::abs(latest->positions[6] - 0.5) < 1e-9);

  const auto timeout_state = arm.query_state(std::chrono::milliseconds(1));
  assert(!timeout_state.has_value());
  assert(fake->written_lines.back() == "[CMD][6]\n");

  auto before = fake->status_sequence();
  fake->feed_line("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:10");
  assert(fake->status_sequence() == before + 1);

  fake->feed_line("[MD][0][0 -180 90 0 0 0 50]");
  fake->feed_line("[MD][2][0 -180 90 0 0 0 60]");
  auto stats = arm.get_frame_stats();
  assert(stats.status_received == 2);
  assert(stats.status_rate_hz == 2.0);
  assert(stats.master_data_received == 2);
  assert(stats.master_data_gap_count == 1);
  assert(stats.master_data_rate_hz == 2.0);

  arm.reset_frame_stats();
  stats = arm.get_frame_stats();
  assert(stats.status_received == 0);
  assert(stats.status_rate_hz == 0.0);
  assert(stats.master_data_received == 0);
  assert(stats.master_data_gap_count == 0);
  assert(stats.master_data_rate_hz == 0.0);
  return 0;
}
