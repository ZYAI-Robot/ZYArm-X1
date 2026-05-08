#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "zyarm_hardware_interface/shell_protocol.hpp"

namespace zyarm_hardware_interface
{

struct SerialConfig
{
  std::string port{"/dev/ttyUSB0"};
  int baud_rate{230400};
  std::chrono::milliseconds read_timeout{20};
  std::chrono::milliseconds write_timeout{20};
  bool reset_rts_dtr{false};
  std::chrono::milliseconds reset_rts_dtr_quiet{0};
  std::chrono::milliseconds activation_status_timeout{1000};
  std::chrono::milliseconds status_stale_warn{100};
  std::chrono::milliseconds status_stale_error{1000};
  std::chrono::milliseconds stale_log_period{2000};
};

class LineIo
{
public:
  virtual ~LineIo() = default;
  virtual bool open(const SerialConfig & config, std::string * error) = 0;
  virtual void close() = 0;
  virtual bool is_open() const = 0;
  virtual bool write_line(const std::string & line, std::string * error) = 0;
  virtual bool read_line(
    std::string & line, std::chrono::milliseconds timeout, std::string * error) = 0;
};

class PosixLineIo : public LineIo
{
public:
  PosixLineIo() = default;
  ~PosixLineIo() override;

  bool open(const SerialConfig & config, std::string * error) override;
  void close() override;
  bool is_open() const override;
  bool write_line(const std::string & line, std::string * error) override;
  bool read_line(
    std::string & line, std::chrono::milliseconds timeout, std::string * error) override;

private:
  int fd_{-1};
  std::chrono::milliseconds write_timeout_{20};
  std::string rx_buffer_;
};

class SerialTransport
{
public:
  explicit SerialTransport(std::unique_ptr<LineIo> io = std::make_unique<PosixLineIo>());
  ~SerialTransport();

  bool open(const SerialConfig & config, std::string * error);
  void close();
  bool is_open() const;
  bool write_line(const std::string & line, std::string * error);

  std::optional<StatusFrame> latest_status() const;
  std::optional<StatusFrame> wait_for_status_after(
    std::chrono::steady_clock::time_point baseline,
    std::chrono::milliseconds timeout) const;

private:
  void receive_loop();
  void update_status(const StatusFrame & frame);

  SerialConfig config_;
  std::unique_ptr<LineIo> io_;
  std::atomic<bool> running_{false};
  std::thread rx_thread_;

  mutable std::mutex status_mutex_;
  mutable std::condition_variable status_cv_;
  std::optional<StatusFrame> latest_status_;
};

}  // namespace zyarm_hardware_interface
