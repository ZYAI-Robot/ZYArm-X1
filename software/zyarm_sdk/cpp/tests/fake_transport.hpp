#pragma once

#include <condition_variable>
#include <deque>
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
  void close() override { connected_ = false; }
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
    written_lines.push_back(zyarm_sdk::format_command(command_id, params));
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

  void feed_line(const std::string & line)
  {
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
  bool connected_{false};
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::optional<zyarm_sdk::StatusFrame> latest_status_;
  std::optional<zyarm_sdk::MasterDataFrame> latest_master_data_;
  std::uint64_t status_sequence_{0};
  std::uint64_t master_data_sequence_{0};
  zyarm_sdk::ArmFrameStats frame_stats_;
  std::optional<int> last_master_frame_id_;
  std::deque<zyarm_sdk::Clock::time_point> status_rate_timestamps_;
  std::deque<zyarm_sdk::Clock::time_point> master_rate_timestamps_;
};
