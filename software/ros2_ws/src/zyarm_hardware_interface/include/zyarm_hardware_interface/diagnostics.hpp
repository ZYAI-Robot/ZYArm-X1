#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>

namespace zyarm_hardware_interface
{

struct DiagnosticsSnapshot
{
  std::uint64_t cmd36_sent{0};
  std::uint64_t status_received{0};
  std::uint64_t status_missed{0};
  std::uint64_t serial_write_errors{0};
  double latest_status_age_ms{-1.0};
  double max_write_duration_ms{0.0};
};

class Diagnostics
{
public:
  void record_cmd36_sent(std::chrono::steady_clock::duration write_duration);
  void record_status_received();
  void record_status_missed();
  void record_serial_write_error();

  DiagnosticsSnapshot snapshot(
    std::chrono::steady_clock::time_point now,
    std::optional<std::chrono::steady_clock::time_point> latest_status_at) const;

private:
  std::atomic<std::uint64_t> cmd36_sent_{0};
  std::atomic<std::uint64_t> status_received_{0};
  std::atomic<std::uint64_t> status_missed_{0};
  std::atomic<std::uint64_t> serial_write_errors_{0};
  std::atomic<std::uint64_t> max_write_duration_us_{0};
};

}  // namespace zyarm_hardware_interface
