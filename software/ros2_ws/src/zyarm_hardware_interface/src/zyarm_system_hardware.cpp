#include "zyarm_hardware_interface/zyarm_system_hardware.hpp"

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "zyarm_hardware_interface/shell_protocol.hpp"

namespace zyarm_hardware_interface
{

namespace
{
std::string get_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  const std::string & fallback)
{
  auto iter = params.find(key);
  if (iter == params.end() || iter->second.empty()) {
    return fallback;
  }
  return iter->second;
}

int get_int_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  int fallback)
{
  auto iter = params.find(key);
  if (iter == params.end() || iter->second.empty()) {
    return fallback;
  }
  return std::stoi(iter->second);
}

bool get_bool_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  bool fallback)
{
  auto iter = params.find(key);
  if (iter == params.end() || iter->second.empty()) {
    return fallback;
  }
  const auto value = iter->second;
  return value == "true" || value == "True" || value == "1";
}

std::chrono::milliseconds get_ms_param(
  const std::unordered_map<std::string, std::string> & params,
  const std::string & key,
  int fallback_ms)
{
  return std::chrono::milliseconds(get_int_param(params, key, fallback_ms));
}

bool has_only_position_interface(const std::vector<hardware_interface::InterfaceInfo> & interfaces)
{
  return interfaces.size() == 1 && interfaces.front().name == hardware_interface::HW_IF_POSITION;
}
}  // namespace

ZyArmSystemHardware::ZyArmSystemHardware()
: joint_mapping_(JointMappingConfig{})
{
}

ZyArmSystemHardware::CallbackReturn ZyArmSystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  if (!validate_interfaces(get_hardware_info())) {
    return CallbackReturn::ERROR;
  }

  if (!load_parameters(get_hardware_info())) {
    return CallbackReturn::ERROR;
  }

  joint_names_ = JointMapping::expected_joint_names();
  state_positions_.fill(0.0);
  command_positions_.fill(0.0);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ZyArmSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joint_names_.size());
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &state_positions_[index]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> ZyArmSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joint_names_.size());
  for (std::size_t index = 0; index < joint_names_.size(); ++index) {
    interfaces.emplace_back(
      joint_names_[index], hardware_interface::HW_IF_POSITION, &command_positions_[index]);
  }
  return interfaces;
}

ZyArmSystemHardware::CallbackReturn ZyArmSystemHardware::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (transport_ == nullptr) {
    transport_ = std::make_unique<SerialTransport>();
  }

  std::string error;
  if (!transport_->open(serial_config_, &error)) {
    RCLCPP_ERROR(get_logger(), "Failed to open ZyArm serial transport: %s", error.c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

ZyArmSystemHardware::CallbackReturn ZyArmSystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (transport_ == nullptr || !transport_->is_open()) {
    RCLCPP_ERROR(get_logger(), "Cannot activate ZyArm hardware: serial transport is not open");
    return CallbackReturn::ERROR;
  }

  auto latest = transport_->latest_status();
  const auto baseline = latest.has_value() ? latest->received_at : std::chrono::steady_clock::time_point{};

  std::array<double, kJointCount> no_change{};
  no_change.fill(kNoChangeSentinel);
  const auto command = format_joint_io_fast_command(no_change);
  std::string error;
  const auto write_started = std::chrono::steady_clock::now();
  if (!transport_->write_line(command, &error)) {
    diagnostics_.record_serial_write_error();
    RCLCPP_ERROR(get_logger(), "Failed to seed ZyArm state with no-change CMD36: %s", error.c_str());
    return CallbackReturn::ERROR;
  }
  diagnostics_.record_cmd36_sent(std::chrono::steady_clock::now() - write_started);

  latest = transport_->wait_for_status_after(baseline, serial_config_.activation_status_timeout);
  if (!latest.has_value()) {
    RCLCPP_ERROR(
      get_logger(), "Timed out waiting for initial ZyArm STATUS during activation");
    return CallbackReturn::ERROR;
  }

  state_positions_ = joint_mapping_.hardware_to_ros(latest->hardware_positions);
  command_positions_ = state_positions_;
  last_consumed_status_at_ = latest->received_at;
  diagnostics_.record_status_received();
  has_state_ = true;
  active_ = true;
  return CallbackReturn::SUCCESS;
}

ZyArmSystemHardware::CallbackReturn ZyArmSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  command_positions_ = state_positions_;
  active_ = false;
  return CallbackReturn::SUCCESS;
}

ZyArmSystemHardware::CallbackReturn ZyArmSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_ = false;
  has_state_ = false;
  if (transport_ != nullptr) {
    transport_->close();
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type ZyArmSystemHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const auto now = std::chrono::steady_clock::now();
  auto latest = transport_ != nullptr ? transport_->latest_status() : std::nullopt;
  if (latest.has_value() && latest->received_at > last_consumed_status_at_) {
    state_positions_ = joint_mapping_.hardware_to_ros(latest->hardware_positions);
    last_consumed_status_at_ = latest->received_at;
    has_state_ = true;
    diagnostics_.record_status_received();
    return hardware_interface::return_type::OK;
  }

  if (active_ && has_state_) {
    diagnostics_.record_status_missed();
    log_stale_status_if_needed(now);
    if (latest.has_value() && now - latest->received_at > serial_config_.status_stale_error) {
      RCLCPP_ERROR(get_logger(), "ZyArm STATUS stale beyond error threshold");
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ZyArmSystemHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_) {
    return hardware_interface::return_type::OK;
  }
  if (transport_ == nullptr || !transport_->is_open()) {
    diagnostics_.record_serial_write_error();
    return hardware_interface::return_type::ERROR;
  }

  const auto hardware_positions = joint_mapping_.ros_to_hardware(command_positions_);
  const auto command = format_joint_io_fast_command(hardware_positions);
  const auto started = std::chrono::steady_clock::now();
  std::string error;
  if (!transport_->write_line(command, &error)) {
    diagnostics_.record_serial_write_error();
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000, "Failed to write ZyArm CMD36: %s", error.c_str());
    return hardware_interface::return_type::ERROR;
  }
  diagnostics_.record_cmd36_sent(std::chrono::steady_clock::now() - started);
  return hardware_interface::return_type::OK;
}

void ZyArmSystemHardware::set_transport_for_testing(std::unique_ptr<SerialTransport> transport)
{
  transport_ = std::move(transport);
}

const std::array<double, kJointCount> & ZyArmSystemHardware::state_positions_for_testing() const
{
  return state_positions_;
}

const std::array<double, kJointCount> & ZyArmSystemHardware::command_positions_for_testing() const
{
  return command_positions_;
}

bool ZyArmSystemHardware::validate_interfaces(const hardware_interface::HardwareInfo & info) const
{
  std::vector<std::string> names;
  names.reserve(info.joints.size());
  for (const auto & joint : info.joints) {
    names.push_back(joint.name);
  }
  if (!JointMapping::has_expected_joint_names(names)) {
    RCLCPP_ERROR(get_logger(), "ZyArm real hardware requires joints joint0..joint6 in order");
    return false;
  }

  for (const auto & joint : info.joints) {
    if (!has_only_position_interface(joint.command_interfaces)) {
      RCLCPP_ERROR(
        get_logger(), "Joint %s must expose only position command interface", joint.name.c_str());
      return false;
    }
    if (!has_only_position_interface(joint.state_interfaces)) {
      RCLCPP_ERROR(
        get_logger(), "Joint %s must expose only position state interface", joint.name.c_str());
      return false;
    }
  }
  return true;
}

bool ZyArmSystemHardware::load_parameters(const hardware_interface::HardwareInfo & info)
{
  try {
    const auto & params = info.hardware_parameters;
    serial_config_.port = get_param(params, "port", serial_config_.port);
    serial_config_.baud_rate = get_int_param(
      params, "baud_rate", get_int_param(params, "baudrate", serial_config_.baud_rate));
    serial_config_.read_timeout = get_ms_param(params, "read_timeout_ms", 20);
    serial_config_.write_timeout = get_ms_param(params, "write_timeout_ms", 20);
    serial_config_.reset_rts_dtr = get_bool_param(params, "reset_rts_dtr", false);
    serial_config_.reset_rts_dtr_quiet = get_ms_param(params, "reset_rts_dtr_quiet_ms", 0);
    serial_config_.activation_status_timeout = get_ms_param(
      params, "activation_status_timeout_ms", 1000);
    serial_config_.status_stale_warn = get_ms_param(params, "status_stale_warn_ms", 100);
    serial_config_.status_stale_error = get_ms_param(params, "status_stale_error_ms", 1000);
    serial_config_.stale_log_period = get_ms_param(params, "stale_log_period_ms", 2000);
    joint_mapping_ = JointMapping(JointMapping::config_from_parameters(params));
  } catch (const std::exception & exc) {
    RCLCPP_ERROR(get_logger(), "Invalid ZyArm hardware parameters: %s", exc.what());
    return false;
  }
  return true;
}

void ZyArmSystemHardware::log_stale_status_if_needed(std::chrono::steady_clock::time_point now)
{
  auto latest = transport_ != nullptr ? transport_->latest_status() : std::nullopt;
  if (!latest.has_value()) {
    return;
  }
  if (now - latest->received_at <= serial_config_.status_stale_warn) {
    return;
  }
  if (last_stale_log_at_.time_since_epoch().count() != 0 &&
    now - last_stale_log_at_ < serial_config_.stale_log_period) {
    return;
  }

  const auto snapshot = diagnostics_.snapshot(now, latest->received_at);
  RCLCPP_WARN(
    get_logger(),
    "ZyArm STATUS stale: age=%.1fms cmd36_sent=%lu status_received=%lu "
    "status_missed=%lu serial_write_errors=%lu max_write=%.2fms",
    snapshot.latest_status_age_ms,
    static_cast<unsigned long>(snapshot.cmd36_sent),
    static_cast<unsigned long>(snapshot.status_received),
    static_cast<unsigned long>(snapshot.status_missed),
    static_cast<unsigned long>(snapshot.serial_write_errors),
    snapshot.max_write_duration_ms);
  last_stale_log_at_ = now;
}

}  // namespace zyarm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  zyarm_hardware_interface::ZyArmSystemHardware, hardware_interface::SystemInterface)
