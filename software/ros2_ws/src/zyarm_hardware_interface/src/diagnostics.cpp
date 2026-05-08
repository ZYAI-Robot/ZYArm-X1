#include "zyarm_hardware_interface/diagnostics.hpp"

#include <algorithm>

namespace zyarm_hardware_interface
{

void Diagnostics::record_cmd36_sent(std::chrono::steady_clock::duration write_duration)
{
  cmd36_sent_.fetch_add(1, std::memory_order_relaxed);
  const auto micros = static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(write_duration).count());

  auto current = max_write_duration_us_.load(std::memory_order_relaxed);
  while (micros > current &&
    !max_write_duration_us_.compare_exchange_weak(
      current, micros, std::memory_order_relaxed, std::memory_order_relaxed)) {
  }
}

void Diagnostics::record_status_received()
{
  status_received_.fetch_add(1, std::memory_order_relaxed);
}

void Diagnostics::record_status_missed()
{
  status_missed_.fetch_add(1, std::memory_order_relaxed);
}

void Diagnostics::record_serial_write_error()
{
  serial_write_errors_.fetch_add(1, std::memory_order_relaxed);
}

DiagnosticsSnapshot Diagnostics::snapshot(
  std::chrono::steady_clock::time_point now,
  std::optional<std::chrono::steady_clock::time_point> latest_status_at) const
{
  DiagnosticsSnapshot result;
  result.cmd36_sent = cmd36_sent_.load(std::memory_order_relaxed);
  result.status_received = status_received_.load(std::memory_order_relaxed);
  result.status_missed = status_missed_.load(std::memory_order_relaxed);
  result.serial_write_errors = serial_write_errors_.load(std::memory_order_relaxed);
  result.max_write_duration_ms =
    static_cast<double>(max_write_duration_us_.load(std::memory_order_relaxed)) / 1000.0;
  if (latest_status_at.has_value()) {
    result.latest_status_age_ms = static_cast<double>(
      std::chrono::duration_cast<std::chrono::microseconds>(now - *latest_status_at).count()) /
      1000.0;
  }
  return result;
}

}  // namespace zyarm_hardware_interface
