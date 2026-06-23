#include <cassert>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <cmath>
#include <iterator>
#include <memory>
#include <string>
#include <thread>

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

  fake->feed_line("[SERVO_TEMP] S1:30 S2:31");
  auto stale_temps = arm.query_servo_temperatures(std::chrono::milliseconds(1));
  assert(!stale_temps.has_value());
  assert(fake->written_lines.back() == "[CMD][6][1]\n");

  std::thread temp_feeder([fake]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    fake->feed_line("[SERVO_TEMP] S1:32 S2:30 S9:29");
  });
  auto temps = arm.query_servo_temperatures(std::chrono::milliseconds(100));
  temp_feeder.join();
  assert(temps.has_value());
  assert(temps->temperatures_c.at(1) == 32);
  assert(temps->temperatures_c.at(9) == 29);
  assert(temps->raw_line == "[SERVO_TEMP] S1:32 S2:30 S9:29");

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

  const std::string log_path = "test_serial_log.txt";
  std::remove(log_path.c_str());
  auto before_log_count = fake->written_line_count();
  arm.set_speed(10);
  assert(before_log_count + 1 == fake->written_line_count());
  assert(!std::ifstream(log_path).good());

  arm.enable_serial_log(log_path, true, true, std::chrono::milliseconds(1));
  arm.set_speed(12);
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  fake->feed_line("UNPARSED AFTER ENABLE");
  arm.flush_serial_log();
  std::ifstream log_file(log_path);
  std::string log_text(
    (std::istreambuf_iterator<char>(log_file)),
    std::istreambuf_iterator<char>());
  assert(log_text.find(" TX [CMD][11][12]") != std::string::npos);
  assert(log_text.find(" RX UNPARSED AFTER ENABLE") != std::string::npos);

  arm.disable_serial_log();
  const auto disabled_log_text = log_text;
  arm.set_speed(13);
  std::ifstream log_file_after_disable(log_path);
  log_text.assign(
    (std::istreambuf_iterator<char>(log_file_after_disable)),
    std::istreambuf_iterator<char>());
  assert(log_text == disabled_log_text);
  std::remove(log_path.c_str());
  return 0;
}
