#pragma once

#include <condition_variable>
#include <deque>
#include <fstream>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "zyarm_sdk/errors.hpp"
#include "zyarm_sdk/protocol.hpp"
#include "zyarm_sdk/transport.hpp"

class FakeTransport : public zyarm_sdk::Transport
{
public:
  void connect() override { connected_ = true; }
  void close() override
  {
    connected_ = false;
    disable_serial_log();
  }
  bool is_connected() const override { return connected_; }

  bool send_command(
    int command_id,
    const std::vector<double> & params = {},
    bool wait_ack = false,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(0)) override
  {
    if (!connected_) {
      throw zyarm_sdk::TransportError("fake transport is not connected");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    command_ids.push_back(command_id);
    wait_acks.push_back(wait_ack);
    timeouts.push_back(timeout);
    const auto line = zyarm_sdk::format_command(command_id, params);
    written_lines.push_back(line);
    write_serial_log("TX", line);
    return true;
  }

  std::optional<zyarm_sdk::StatusFrame> latest_status() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_status_;
  }

  std::optional<zyarm_sdk::StatusFrame> wait_for_status_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = zyarm_sdk::Clock::now() + timeout;
    while (!latest_status_.has_value() || latest_status_->sequence <= sequence) {
      if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
        return std::nullopt;
      }
    }
    return latest_status_;
  }

  std::optional<zyarm_sdk::MasterDataFrame> latest_master_data() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_master_data_;
  }

  std::optional<zyarm_sdk::ServoTemperatureFrame> wait_for_servo_temperatures_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = zyarm_sdk::Clock::now() + timeout;
    while (!latest_servo_temperatures_.has_value() ||
           latest_servo_temperatures_->sequence <= sequence) {
      if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
        return std::nullopt;
      }
    }
    return latest_servo_temperatures_;
  }

  std::optional<zyarm_sdk::MasterDataFrame> wait_for_master_data_after(
    std::uint64_t sequence,
    std::chrono::milliseconds timeout) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    const auto deadline = zyarm_sdk::Clock::now() + timeout;
    while (!latest_master_data_.has_value() || latest_master_data_->sequence <= sequence) {
      if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
        return std::nullopt;
      }
    }
    return latest_master_data_;
  }

  std::uint64_t status_sequence() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_sequence_;
  }

  std::uint64_t master_data_sequence() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return master_data_sequence_;
  }

  std::uint64_t servo_temperature_sequence() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return servo_temperature_sequence_;
  }

  zyarm_sdk::ArmFrameStats get_frame_stats() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto stats = frame_stats_;
    stats.master_data_rate_hz = static_cast<double>(master_rate_timestamps_.size());
    stats.status_rate_hz = static_cast<double>(status_rate_timestamps_.size());
    return stats;
  }

  void reset_frame_stats() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_stats_ = zyarm_sdk::ArmFrameStats{};
    last_master_frame_id_.reset();
    status_rate_timestamps_.clear();
    master_rate_timestamps_.clear();
  }

  void enable_serial_log(
    const std::string & path,
    bool include_tx = true,
    bool include_rx = true,
    std::optional<std::chrono::milliseconds> flush_interval = std::nullopt) override
  {
    if (flush_interval.has_value() && flush_interval->count() < 0) {
      throw zyarm_sdk::TransportError("serial log flush interval must be non-negative");
    }
    std::lock_guard<std::mutex> lock(log_mutex_);
    close_serial_log_locked();
    serial_log_.open(path, std::ios::out | std::ios::app);
    if (!serial_log_.is_open()) {
      throw zyarm_sdk::TransportError("failed to open fake serial log");
    }
    include_tx_ = include_tx;
    include_rx_ = include_rx;
    if (flush_interval.has_value() && flush_interval->count() > 0) {
      flush_interval_ = flush_interval;
    } else {
      flush_interval_.reset();
    }
    last_flush_at_ = zyarm_sdk::Clock::now();
  }

  void flush_serial_log() override
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (serial_log_.is_open()) {
      serial_log_.flush();
      last_flush_at_ = zyarm_sdk::Clock::now();
    }
  }

  void disable_serial_log() override
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    close_serial_log_locked();
  }

  void feed_line(const std::string & line)
  {
    write_serial_log("RX", line);
    std::lock_guard<std::mutex> lock(mutex_);
    if (auto status = zyarm_sdk::parse_status_line(line, status_sequence_ + 1)) {
      status->sequence = ++status_sequence_;
      latest_status_ = *status;
      frame_stats_.status_received++;
      status_rate_timestamps_.push_back(status->received_at);
      cv_.notify_all();
      return;
    }
    if (auto md = zyarm_sdk::parse_master_data_line(line, master_data_sequence_ + 1)) {
      md->sequence = ++master_data_sequence_;
      latest_master_data_ = *md;
      const int frame_id = ((md->frame_id % 10) + 10) % 10;
      if (last_master_frame_id_.has_value()) {
        frame_stats_.master_data_gap_count += static_cast<std::uint64_t>(
          (frame_id - *last_master_frame_id_ - 1 + 10) % 10);
      }
      last_master_frame_id_ = frame_id;
      frame_stats_.master_data_received++;
      master_rate_timestamps_.push_back(md->received_at);
      cv_.notify_all();
      return;
    }
    if (auto temp = zyarm_sdk::parse_servo_temperature_line(line, servo_temperature_sequence_ + 1)) {
      temp->sequence = ++servo_temperature_sequence_;
      latest_servo_temperatures_ = *temp;
      cv_.notify_all();
    }
  }

  std::size_t written_line_count() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return written_lines.size();
  }

  std::vector<std::string> written_lines;
  std::vector<int> command_ids;
  std::vector<bool> wait_acks;
  std::vector<std::chrono::milliseconds> timeouts;

private:
  void write_serial_log(const std::string & direction, std::string line)
  {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (!serial_log_.is_open()) {
      return;
    }
    if (direction == "TX" && !include_tx_) {
      return;
    }
    if (direction == "RX" && !include_rx_) {
      return;
    }
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r')) {
      line.pop_back();
    }
    serial_log_ << "1970-01-01T00:00:00.000 " << direction << " " << line << "\n";
    if (flush_interval_.has_value()) {
      const auto now = zyarm_sdk::Clock::now();
      if (now - last_flush_at_ >= *flush_interval_) {
        serial_log_.flush();
        last_flush_at_ = now;
      }
    }
  }

  void close_serial_log_locked()
  {
    if (serial_log_.is_open()) {
      serial_log_.flush();
      serial_log_.close();
    }
  }

  bool connected_{false};
  mutable std::mutex mutex_;
  mutable std::mutex log_mutex_;
  std::condition_variable cv_;
  std::optional<zyarm_sdk::StatusFrame> latest_status_;
  std::optional<zyarm_sdk::MasterDataFrame> latest_master_data_;
  std::optional<zyarm_sdk::ServoTemperatureFrame> latest_servo_temperatures_;
  std::uint64_t status_sequence_{0};
  std::uint64_t master_data_sequence_{0};
  std::uint64_t servo_temperature_sequence_{0};
  zyarm_sdk::ArmFrameStats frame_stats_;
  std::optional<int> last_master_frame_id_;
  std::deque<zyarm_sdk::Clock::time_point> status_rate_timestamps_;
  std::deque<zyarm_sdk::Clock::time_point> master_rate_timestamps_;
  std::ofstream serial_log_;
  bool include_tx_{true};
  bool include_rx_{true};
  std::optional<std::chrono::milliseconds> flush_interval_;
  zyarm_sdk::Clock::time_point last_flush_at_{zyarm_sdk::Clock::now()};
};
