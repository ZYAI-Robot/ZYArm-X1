#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "zyarm_sdk/config.hpp"
#include "zyarm_sdk/protocol.hpp"

namespace zyarm_sdk
{

class Transport
{
public:
  virtual ~Transport() = default;

  virtual void connect() = 0;
  virtual void close() = 0;
  virtual bool is_connected() const = 0;
  virtual bool send_command(
    int command_id,
    const std::vector<double> & params = {},
    bool wait_ack = false,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) = 0;
  virtual std::optional<StatusFrame> latest_status() const = 0;
  virtual std::optional<StatusFrame> wait_for_status_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) = 0;
  virtual std::optional<MasterDataFrame> latest_master_data() const = 0;
  virtual std::optional<MasterDataFrame> wait_for_master_data_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) = 0;
  virtual std::optional<ServoTemperatureFrame> wait_for_servo_temperatures_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) = 0;
  virtual std::uint64_t status_sequence() const = 0;
  virtual std::uint64_t master_data_sequence() const = 0;
  virtual std::uint64_t servo_temperature_sequence() const = 0;
  virtual ArmFrameStats get_frame_stats() const = 0;
  virtual void reset_frame_stats() = 0;
  virtual void enable_serial_log(
    const std::string & path,
    bool include_tx = true,
    bool include_rx = true,
    std::optional<std::chrono::milliseconds> flush_interval = std::nullopt) = 0;
  virtual void flush_serial_log() = 0;
  virtual void disable_serial_log() = 0;
};

class SerialTransport : public Transport
{
public:
  explicit SerialTransport(ZyArmConfig config);
  ~SerialTransport() override;

  void connect() override;
  void close() override;
  bool is_connected() const override;
  bool send_command(
    int command_id,
    const std::vector<double> & params = {},
    bool wait_ack = false,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) override;
  std::optional<StatusFrame> latest_status() const override;
  std::optional<StatusFrame> wait_for_status_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) override;
  std::optional<MasterDataFrame> latest_master_data() const override;
  std::optional<MasterDataFrame> wait_for_master_data_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) override;
  std::optional<ServoTemperatureFrame> wait_for_servo_temperatures_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) override;
  std::uint64_t status_sequence() const override;
  std::uint64_t master_data_sequence() const override;
  std::uint64_t servo_temperature_sequence() const override;
  ArmFrameStats get_frame_stats() const override;
  void reset_frame_stats() override;
  void enable_serial_log(
    const std::string & path,
    bool include_tx = true,
    bool include_rx = true,
    std::optional<std::chrono::milliseconds> flush_interval = std::nullopt) override;
  void flush_serial_log() override;
  void disable_serial_log() override;

  void feed_line_for_test(const std::string & line);

private:
  void rx_loop();
  void handle_rx_line(const std::string & line);
  bool wait_for_ack(int command_id, std::chrono::milliseconds timeout);
  void record_status_stats(const StatusFrame & frame);
  void record_master_data_stats(const MasterDataFrame & frame);
  void prune_rate_windows(Clock::time_point now) const;
  void write_serial_log(const std::string & direction, const std::string & line);
  void close_serial_log_locked();

  ZyArmConfig config_;
  mutable std::mutex mutex_;
  mutable std::mutex serial_log_mutex_;
  std::condition_variable cv_;
  std::map<int, bool> ack_state_;
  std::optional<StatusFrame> latest_status_;
  std::optional<MasterDataFrame> latest_master_data_;
  std::optional<ServoTemperatureFrame> latest_servo_temperatures_;
  std::uint64_t status_sequence_{0};
  std::uint64_t master_data_sequence_{0};
  std::uint64_t servo_temperature_sequence_{0};
  ArmFrameStats frame_stats_;
  std::optional<int> stats_last_master_frame_id_;
  mutable std::deque<Clock::time_point> status_rate_timestamps_;
  mutable std::deque<Clock::time_point> master_rate_timestamps_;
  std::ofstream serial_log_stream_;
  bool serial_log_include_tx_{true};
  bool serial_log_include_rx_{true};
  std::optional<std::chrono::milliseconds> serial_log_flush_interval_;
  Clock::time_point serial_log_last_flush_at_{Clock::now()};
  bool connected_{false};
  bool stop_rx_{false};
  std::thread rx_thread_;
  std::string rx_buffer_;

#ifdef _WIN32
  void * handle_{nullptr};
#else
  int fd_{-1};
#endif
};

}  // namespace zyarm_sdk
