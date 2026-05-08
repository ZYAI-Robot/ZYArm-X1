#include "zyarm_sdk/transport.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>

#include "zyarm_sdk/errors.hpp"

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace zyarm_sdk
{

namespace
{
constexpr auto kFrameRateWindow = std::chrono::seconds(1);

#ifndef _WIN32
speed_t baud_to_constant(int baudrate)
{
  switch (baudrate) {
    case 9600:
      return B9600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B1000000
    case 1000000:
      return B1000000;
#endif
    default:
#ifdef B230400
      return B230400;
#else
#ifdef B1000000
      return B1000000;
#else
      return B115200;
#endif
#endif
  }
}
#endif
}  // namespace

SerialTransport::SerialTransport(ZyArmConfig config) : config_(std::move(config)) {}

SerialTransport::~SerialTransport()
{
  close();
}

void SerialTransport::connect()
{
  if (is_connected()) {
    return;
  }
#ifdef _WIN32
  std::string port = config_.port;
  if (port.rfind("COM", 0) == 0 && port.size() > 4) {
    port = "\\\\.\\" + port;
  }
  HANDLE handle = CreateFileA(
    port.c_str(),
    GENERIC_READ | GENERIC_WRITE,
    0,
    nullptr,
    OPEN_EXISTING,
    0,
    nullptr);
  if (handle == INVALID_HANDLE_VALUE) {
    throw TransportError("failed to open Windows serial port");
  }
  DCB dcb{};
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(handle, &dcb)) {
    CloseHandle(handle);
    throw TransportError("failed to read Windows serial settings");
  }
  dcb.BaudRate = static_cast<DWORD>(config_.baudrate);
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  if (!SetCommState(handle, &dcb)) {
    CloseHandle(handle);
    throw TransportError("failed to configure Windows serial settings");
  }
  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = static_cast<DWORD>(config_.timeout.count());
  timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(config_.timeout.count());
  timeouts.WriteTotalTimeoutConstant = static_cast<DWORD>(config_.write_timeout.count());
  SetCommTimeouts(handle, &timeouts);
  handle_ = handle;
#else
  fd_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw TransportError("failed to open serial port: " + config_.port);
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw TransportError("failed to read serial settings");
  }
  cfmakeraw(&tty);
  const speed_t baud = baud_to_constant(config_.baudrate);
  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
#ifdef CRTSCTS
  tty.c_cflag &= ~CRTSCTS;
#endif
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = static_cast<unsigned char>(std::max<long>(1, config_.timeout.count() / 100));
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    throw TransportError("failed to apply serial settings");
  }
#endif
  {
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = true;
    stop_rx_ = false;
  }
  rx_thread_ = std::thread(&SerialTransport::rx_loop, this);
}

void SerialTransport::close()
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_rx_ = true;
    connected_ = false;
  }
  cv_.notify_all();
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
#ifdef _WIN32
  if (handle_ != nullptr) {
    CloseHandle(static_cast<HANDLE>(handle_));
    handle_ = nullptr;
  }
#else
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#endif
}

bool SerialTransport::is_connected() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return connected_;
}

bool SerialTransport::send_command(
  int command_id,
  const std::vector<double> & params,
  bool wait_ack,
  std::chrono::milliseconds timeout)
{
  const std::string line = format_command(command_id, params);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!connected_) {
      throw TransportError("serial port is not open");
    }
    ack_state_.erase(command_id);
  }
#ifdef _WIN32
  DWORD written = 0;
  if (!WriteFile(static_cast<HANDLE>(handle_), line.data(), static_cast<DWORD>(line.size()), &written, nullptr)) {
    throw TransportError("failed to write Windows serial command");
  }
#else
  const auto written = ::write(fd_, line.data(), line.size());
  if (written < 0 || static_cast<std::size_t>(written) != line.size()) {
    throw TransportError("failed to write serial command");
  }
#endif
  if (!wait_ack) {
    return true;
  }
  return wait_for_ack(command_id, timeout.count() > 0 ? timeout : config_.ack_timeout);
}

std::optional<StatusFrame> SerialTransport::latest_status() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_status_;
}

std::optional<StatusFrame> SerialTransport::wait_for_status_after(
  std::uint64_t sequence,
  std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const auto deadline = Clock::now() + timeout;
  while (!latest_status_.has_value() || latest_status_->sequence <= sequence) {
    if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      return std::nullopt;
    }
  }
  return latest_status_;
}

std::optional<MasterDataFrame> SerialTransport::latest_master_data() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_master_data_;
}

std::optional<MasterDataFrame> SerialTransport::wait_for_master_data_after(
  std::uint64_t sequence,
  std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const auto deadline = Clock::now() + timeout;
  while (!latest_master_data_.has_value() || latest_master_data_->sequence <= sequence) {
    if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      return std::nullopt;
    }
  }
  return latest_master_data_;
}

std::uint64_t SerialTransport::status_sequence() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_sequence_;
}

std::uint64_t SerialTransport::master_data_sequence() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return master_data_sequence_;
}

ArmFrameStats SerialTransport::get_frame_stats() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  prune_rate_windows(Clock::now());
  auto stats = frame_stats_;
  stats.master_data_rate_hz = static_cast<double>(master_rate_timestamps_.size());
  stats.status_rate_hz = static_cast<double>(status_rate_timestamps_.size());
  return stats;
}

void SerialTransport::reset_frame_stats()
{
  std::lock_guard<std::mutex> lock(mutex_);
  frame_stats_ = ArmFrameStats{};
  stats_last_master_frame_id_.reset();
  status_rate_timestamps_.clear();
  master_rate_timestamps_.clear();
}

void SerialTransport::feed_line_for_test(const std::string & line)
{
  handle_rx_line(line);
}

void SerialTransport::rx_loop()
{
  char buffer[128];
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_rx_) {
        return;
      }
    }
#ifdef _WIN32
    DWORD read_count = 0;
    const BOOL ok = ReadFile(static_cast<HANDLE>(handle_), buffer, sizeof(buffer), &read_count, nullptr);
    if (!ok || read_count == 0) {
      std::this_thread::sleep_for(config_.timeout);
      continue;
    }
    const int count = static_cast<int>(read_count);
#else
    fd_set read_set;
    FD_ZERO(&read_set);
    FD_SET(fd_, &read_set);
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = static_cast<suseconds_t>(config_.timeout.count() * 1000);
    const int ready = select(fd_ + 1, &read_set, nullptr, nullptr, &tv);
    if (ready <= 0) {
      continue;
    }
    const int count = static_cast<int>(::read(fd_, buffer, sizeof(buffer)));
    if (count <= 0) {
      continue;
    }
#endif
    for (int index = 0; index < count; ++index) {
      const char ch = buffer[index];
      if (ch == '\n') {
        const std::string line = rx_buffer_;
        rx_buffer_.clear();
        if (!line.empty()) {
          handle_rx_line(line);
        }
      } else if (ch != '\r') {
        rx_buffer_.push_back(ch);
      }
    }
  }
}

void SerialTransport::handle_rx_line(const std::string & line)
{
  if (auto ack = parse_ack(line)) {
    std::lock_guard<std::mutex> lock(mutex_);
    ack_state_[ack->command_id] = ack->success;
    cv_.notify_all();
    return;
  }
  if (auto status = parse_status_line(line, status_sequence_ + 1)) {
    std::lock_guard<std::mutex> lock(mutex_);
    status->sequence = ++status_sequence_;
    latest_status_ = *status;
    record_status_stats(*status);
    cv_.notify_all();
    return;
  }
  if (auto master = parse_master_data_line(line, master_data_sequence_ + 1)) {
    std::lock_guard<std::mutex> lock(mutex_);
    master->sequence = ++master_data_sequence_;
    latest_master_data_ = *master;
    record_master_data_stats(*master);
    cv_.notify_all();
  }
}

void SerialTransport::record_status_stats(const StatusFrame & frame)
{
  frame_stats_.status_received++;
  status_rate_timestamps_.push_back(frame.received_at);
  prune_rate_windows(frame.received_at);
}

void SerialTransport::record_master_data_stats(const MasterDataFrame & frame)
{
  const int frame_id = ((frame.frame_id % 10) + 10) % 10;
  if (stats_last_master_frame_id_.has_value()) {
    frame_stats_.master_data_gap_count += static_cast<std::uint64_t>(
      (frame_id - *stats_last_master_frame_id_ - 1 + 10) % 10);
  }
  stats_last_master_frame_id_ = frame_id;
  frame_stats_.master_data_received++;
  master_rate_timestamps_.push_back(frame.received_at);
  prune_rate_windows(frame.received_at);
}

void SerialTransport::prune_rate_windows(Clock::time_point now) const
{
  const auto cutoff = now - kFrameRateWindow;
  while (!status_rate_timestamps_.empty() && status_rate_timestamps_.front() < cutoff) {
    status_rate_timestamps_.pop_front();
  }
  while (!master_rate_timestamps_.empty() && master_rate_timestamps_.front() < cutoff) {
    master_rate_timestamps_.pop_front();
  }
}

bool SerialTransport::wait_for_ack(int command_id, std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  const auto deadline = Clock::now() + timeout;
  while (ack_state_.find(command_id) == ack_state_.end()) {
    if (cv_.wait_until(lock, deadline) == std::cv_status::timeout) {
      return false;
    }
  }
  return ack_state_[command_id];
}

}  // namespace zyarm_sdk
