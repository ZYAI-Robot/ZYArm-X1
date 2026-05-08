#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "zyarm_hardware_interface/diagnostics.hpp"
#include "zyarm_hardware_interface/joint_mapping.hpp"
#include "zyarm_hardware_interface/serial_transport.hpp"

namespace zyarm_hardware_interface
{

class ZyArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ZyArmSystemHardware();

  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void set_transport_for_testing(std::unique_ptr<SerialTransport> transport);
  const std::array<double, kJointCount> & state_positions_for_testing() const;
  const std::array<double, kJointCount> & command_positions_for_testing() const;

private:
  bool validate_interfaces(const hardware_interface::HardwareInfo & info) const;
  bool load_parameters(const hardware_interface::HardwareInfo & info);
  void log_stale_status_if_needed(std::chrono::steady_clock::time_point now);

  std::vector<std::string> joint_names_;
  JointMapping joint_mapping_;
  SerialConfig serial_config_;
  Diagnostics diagnostics_;
  std::unique_ptr<SerialTransport> transport_;

  std::array<double, kJointCount> state_positions_{};
  std::array<double, kJointCount> command_positions_{};
  std::chrono::steady_clock::time_point last_consumed_status_at_{};
  std::chrono::steady_clock::time_point last_stale_log_at_{};
  bool has_state_{false};
  bool active_{false};
};

}  // namespace zyarm_hardware_interface
