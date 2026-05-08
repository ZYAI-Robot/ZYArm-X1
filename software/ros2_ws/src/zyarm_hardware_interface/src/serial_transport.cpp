#include "zyarm_hardware_interface/serial_transport.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

namespace zyarm_hardware_interface
{

namespace
{
speed_t baud_to_termios(int baud_rate)
{
  switch (baud_rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
#ifdef B230400
    case 230400:
      return B230400;
#endif
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return 0;
  }
}

std::string errno_message(const std::string & prefix)
{
  return prefix + ": " + std::strerror(errno);
}

timeval to_timeval(std::chrono::milliseconds duration)
{
  if (duration.count() < 0) {
    duration = std::chrono::milliseconds(0);
  }
  timeval tv{};
  tv.tv_sec = static_cast<time_t>(duration.count() / 1000);
  tv.tv_usec = static_cast<suseconds_t>((duration.count() % 1000) * 1000);
  return tv;
}

bool wait_for_fd(int fd, bool write_ready, std::chrono::milliseconds timeout, std::string * error)
{
  fd_set set;
  FD_ZERO(&set);
  FD_SET(fd, &set);
  auto tv = to_timeval(timeout);
  const int ret = select(
    fd + 1,
    write_ready ? nullptr : &set,
    write_ready ? &set : nullptr,
    nullptr,
    &tv);
  if (ret > 0) {
    return true;
  }
  if (ret == 0) {
    return false;
  }
  if (error != nullptr) {
    *error = errno_message(write_ready ? "serial write select failed" : "serial read select failed");
  }
  return false;
}

void set_modem_line(int fd, int line, bool enabled)
{
  int flags = 0;
  if (ioctl(fd, TIOCMGET, &flags) != 0) {
    return;
  }
  if (enabled) {
    flags |= line;
  } else {
    flags &= ~line;
  }
  (void)ioctl(fd, TIOCMSET, &flags);
}
}  // namespace

PosixLineIo::~PosixLineIo()
{
  close();
}

bool PosixLineIo::open(const SerialConfig & config, std::string * error)
{
  if (is_open()) {
    return true;
  }

  const speed_t baud = baud_to_termios(config.baud_rate);
  if (baud == 0) {
    if (error != nullptr) {
      *error = "unsupported baud_rate: " + std::to_string(config.baud_rate);
    }
    return false;
  }

  fd_ = ::open(config.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    if (error != nullptr) {
      *error = errno_message("failed to open " + config.port);
    }
    return false;
  }

  termios options{};
  if (tcgetattr(fd_, &options) != 0) {
    if (error != nullptr) {
      *error = errno_message("tcgetattr failed");
    }
    close();
    return false;
  }

  cfmakeraw(&options);
  cfsetispeed(&options, baud);
  cfsetospeed(&options, baud);
  options.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
  options.c_cflag &= static_cast<tcflag_t>(~PARENB);
  options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  options.c_cflag |= CS8;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    if (error != nullptr) {
      *error = errno_message("tcsetattr failed");
    }
    close();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  write_timeout_ = config.write_timeout;

  if (config.reset_rts_dtr) {
    set_modem_line(fd_, TIOCM_DTR, false);
    set_modem_line(fd_, TIOCM_RTS, true);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    set_modem_line(fd_, TIOCM_RTS, false);
    if (config.reset_rts_dtr_quiet.count() > 0) {
      std::this_thread::sleep_for(config.reset_rts_dtr_quiet);
    }
  }

  return true;
}

void PosixLineIo::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  rx_buffer_.clear();
}

bool PosixLineIo::is_open() const
{
  return fd_ >= 0;
}

bool PosixLineIo::write_line(const std::string & line, std::string * error)
{
  if (!is_open()) {
    if (error != nullptr) {
      *error = "serial port is not open";
    }
    return false;
  }

  std::size_t written = 0;
  const auto deadline = std::chrono::steady_clock::now() + write_timeout_;
  while (written < line.size()) {
    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      if (error != nullptr) {
        *error = "serial write timed out";
      }
      return false;
    }
    if (!wait_for_fd(
        fd_, true,
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now), error)) {
      if (error != nullptr && error->empty()) {
        *error = "serial write timed out";
      }
      return false;
    }

    const ssize_t ret = ::write(fd_, line.data() + written, line.size() - written);
    if (ret < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      if (error != nullptr) {
        *error = errno_message("serial write failed");
      }
      return false;
    }
    written += static_cast<std::size_t>(ret);
  }

  if (tcdrain(fd_) != 0) {
    if (error != nullptr) {
      *error = errno_message("serial tcdrain failed");
    }
    return false;
  }
  return true;
}

bool PosixLineIo::read_line(
  std::string & line, std::chrono::milliseconds timeout, std::string * error)
{
  line.clear();
  if (!is_open()) {
    if (error != nullptr) {
      *error = "serial port is not open";
    }
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (true) {
    const auto newline_pos = rx_buffer_.find('\n');
    if (newline_pos != std::string::npos) {
      line = rx_buffer_.substr(0, newline_pos);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      rx_buffer_.erase(0, newline_pos + 1);
      return true;
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      return false;
    }

    std::string select_error;
    const bool ready = wait_for_fd(
      fd_, false, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now),
      &select_error);
    if (!ready) {
      if (!select_error.empty() && error != nullptr) {
        *error = select_error;
      }
      return false;
    }

    char buffer[128];
    const ssize_t ret = ::read(fd_, buffer, sizeof(buffer));
    if (ret < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        continue;
      }
      if (error != nullptr) {
        *error = errno_message("serial read failed");
      }
      return false;
    }
    if (ret == 0) {
      continue;
    }
    rx_buffer_.append(buffer, static_cast<std::size_t>(ret));
  }
}

SerialTransport::SerialTransport(std::unique_ptr<LineIo> io)
: io_(std::move(io))
{
}

SerialTransport::~SerialTransport()
{
  close();
}

bool SerialTransport::open(const SerialConfig & config, std::string * error)
{
  if (is_open()) {
    return true;
  }
  config_ = config;
  if (!io_->open(config_, error)) {
    return false;
  }
  running_.store(true);
  rx_thread_ = std::thread(&SerialTransport::receive_loop, this);
  return true;
}

void SerialTransport::close()
{
  running_.store(false);
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }
  if (io_) {
    io_->close();
  }
}

bool SerialTransport::is_open() const
{
  return io_ != nullptr && io_->is_open();
}

bool SerialTransport::write_line(const std::string & line, std::string * error)
{
  if (io_ == nullptr) {
    if (error != nullptr) {
      *error = "serial transport has no IO backend";
    }
    return false;
  }
  return io_->write_line(line, error);
}

std::optional<StatusFrame> SerialTransport::latest_status() const
{
  std::lock_guard<std::mutex> lock(status_mutex_);
  return latest_status_;
}

std::optional<StatusFrame> SerialTransport::wait_for_status_after(
  std::chrono::steady_clock::time_point baseline,
  std::chrono::milliseconds timeout) const
{
  std::unique_lock<std::mutex> lock(status_mutex_);
  const auto predicate = [&]() {
      return latest_status_.has_value() && latest_status_->received_at > baseline;
    };
  if (!status_cv_.wait_for(lock, timeout, predicate)) {
    return std::nullopt;
  }
  return latest_status_;
}

void SerialTransport::receive_loop()
{
  while (running_.load()) {
    std::string line;
    std::string error;
    if (!io_->read_line(line, config_.read_timeout, &error)) {
      continue;
    }
    auto frame = parse_status_frame(line);
    if (frame.has_value()) {
      update_status(*frame);
    }
  }
}

void SerialTransport::update_status(const StatusFrame & frame)
{
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    latest_status_ = frame;
  }
  status_cv_.notify_all();
}

}  // namespace zyarm_hardware_interface
