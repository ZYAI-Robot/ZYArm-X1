#include "zyarm_hardware_interface/zyarm_system_hardware.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <sys/stat.h>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace zyarm_hardware_interface
{

namespace
{
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RclcppEnvironment : public ::testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      const char * log_dir = "/tmp/zyarm_hardware_interface_test_logs";
      (void)mkdir(log_dir, 0755);
      setenv("ROS_LOG_DIR", log_dir, 1);
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

class FakeLineIo : public LineIo
{
public:
  bool open(const SerialConfig &, std::string *) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    open_ = true;
    return true;
  }

  void close() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      open_ = false;
    }
    cv_.notify_all();
  }

  bool is_open() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return open_;
  }

  bool write_line(const std::string & line, std::string * error) override
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!write_ok_) {
        if (error != nullptr) {
          *error = "fake write failed";
        }
        return false;
      }
      writes_.push_back(line);
      if (!status_on_write_.empty()) {
        read_lines_.push_back(status_on_write_);
      }
    }
    cv_.notify_all();
    return true;
  }

  bool read_line(std::string & line, std::chrono::milliseconds timeout, std::string *) override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!cv_.wait_for(lock, timeout, [&]() {return !read_lines_.empty() || !open_;})) {
      return false;
    }
    if (read_lines_.empty()) {
      return false;
    }
    line = read_lines_.front();
    read_lines_.pop_front();
    return true;
  }

  void set_status_on_write(const std::string & line)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_on_write_ = line;
  }

  std::vector<std::string> writes() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return writes_;
  }

private:
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::string> read_lines_;
  std::vector<std::string> writes_;
  std::string status_on_write_;
  bool open_{false};
  bool write_ok_{true};
};

hardware_interface::ComponentInfo make_joint(const std::string & name)
{
  hardware_interface::InterfaceInfo position;
  position.name = hardware_interface::HW_IF_POSITION;

  hardware_interface::ComponentInfo joint;
  joint.name = name;
  joint.command_interfaces.push_back(position);
  joint.state_interfaces.push_back(position);
  return joint;
}

hardware_interface::HardwareInfo make_hardware_info()
{
  hardware_interface::HardwareInfo info;
  info.name = "ZyArm2BRealSystem";
  info.type = "system";
  info.rw_rate = 50;
  info.is_async = false;
  info.hardware_plugin_name = "zyarm_hardware_interface/ZyArmSystemHardware";
  info.hardware_parameters = {
    {"port", "fake"},
    {"baud_rate", "230400"},
    {"read_timeout_ms", "2"},
    {"write_timeout_ms", "2"},
    {"activation_status_timeout_ms", "200"},
    {"status_stale_warn_ms", "100"},
    {"status_stale_error_ms", "1000"},
    {"stale_log_period_ms", "2000"},
  };
  for (std::size_t index = 0; index < kJointCount; ++index) {
    info.joints.push_back(make_joint("joint" + std::to_string(index)));
  }
  return info;
}

hardware_interface::HardwareComponentInterfaceParams make_params(
  const hardware_interface::HardwareInfo & info)
{
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  return params;
}

std::unique_ptr<SerialTransport> make_fake_transport(FakeLineIo ** fake)
{
  auto fake_io = std::make_unique<FakeLineIo>();
  *fake = fake_io.get();
  return std::make_unique<SerialTransport>(std::move(fake_io));
}

rclcpp_lifecycle::State lifecycle_state()
{
  return rclcpp_lifecycle::State();
}

rclcpp::Time test_time()
{
  return rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
}

rclcpp::Duration test_period()
{
  return rclcpp::Duration(0, 0);
}

[[maybe_unused]] const auto * const rclcpp_environment =
  ::testing::AddGlobalTestEnvironment(new RclcppEnvironment);
}  // namespace

TEST(ZyArmSystemHardware, RejectsUnsupportedInterfacesAndJoint7)
{
  ZyArmSystemHardware hardware;

  auto info = make_hardware_info();
  info.joints.push_back(make_joint("joint7"));
  EXPECT_EQ(hardware.on_init(make_params(info)), CallbackReturn::ERROR);

  info = make_hardware_info();
  hardware_interface::InterfaceInfo velocity;
  velocity.name = hardware_interface::HW_IF_VELOCITY;
  info.joints[0].state_interfaces.push_back(velocity);
  EXPECT_EQ(hardware.on_init(make_params(info)), CallbackReturn::ERROR);
}

TEST(ZyArmSystemHardware, ExportsPositionOnlyInterfaces)
{
  ZyArmSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_params(make_hardware_info())), CallbackReturn::SUCCESS);

  const auto states = hardware.export_state_interfaces();
  const auto commands = hardware.export_command_interfaces();
  ASSERT_EQ(states.size(), kJointCount);
  ASSERT_EQ(commands.size(), kJointCount);
  for (std::size_t index = 0; index < kJointCount; ++index) {
    EXPECT_EQ(states[index].get_name(), "joint" + std::to_string(index) + "/position");
    EXPECT_EQ(commands[index].get_name(), "joint" + std::to_string(index) + "/position");
  }
}

TEST(ZyArmSystemHardware, ActivationSeedsStateAndCommandWithoutReset)
{
  ZyArmSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_params(make_hardware_info())), CallbackReturn::SUCCESS);

  FakeLineIo * fake = nullptr;
  hardware.set_transport_for_testing(make_fake_transport(&fake));
  fake->set_status_on_write("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:50");

  ASSERT_EQ(hardware.on_configure(lifecycle_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(lifecycle_state()), CallbackReturn::SUCCESS);

  const auto writes = fake->writes();
  ASSERT_EQ(writes.size(), 1u);
  EXPECT_EQ(
    writes.front(),
    "[CMD][36][-999.900 -999.900 -999.900 -999.900 -999.900 -999.900 -999.900]\n");

  const auto & state = hardware.state_positions_for_testing();
  const auto & command = hardware.command_positions_for_testing();
  EXPECT_NEAR(state[0], 0.0, 1e-9);
  EXPECT_NEAR(state[1], 0.0, 1e-9);
  EXPECT_NEAR(state[2], 0.0, 1e-9);
  EXPECT_NEAR(state[6], 0.017, 1e-9);
  EXPECT_EQ(command, state);

  ASSERT_EQ(hardware.on_cleanup(lifecycle_state()), CallbackReturn::SUCCESS);
}

TEST(ZyArmSystemHardware, ReadKeepsPreviousStateWhenStatusIsMissing)
{
  ZyArmSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_params(make_hardware_info())), CallbackReturn::SUCCESS);

  FakeLineIo * fake = nullptr;
  hardware.set_transport_for_testing(make_fake_transport(&fake));
  fake->set_status_on_write("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:0");

  ASSERT_EQ(hardware.on_configure(lifecycle_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(lifecycle_state()), CallbackReturn::SUCCESS);

  const auto before = hardware.state_positions_for_testing();
  EXPECT_EQ(hardware.read(test_time(), test_period()), hardware_interface::return_type::OK);
  EXPECT_EQ(hardware.state_positions_for_testing(), before);

  ASSERT_EQ(hardware.on_cleanup(lifecycle_state()), CallbackReturn::SUCCESS);
}

TEST(ZyArmSystemHardware, WriteGeneratesSingleCmd36AndDeactivateHoldsCurrentState)
{
  ZyArmSystemHardware hardware;
  ASSERT_EQ(hardware.on_init(make_params(make_hardware_info())), CallbackReturn::SUCCESS);

  FakeLineIo * fake = nullptr;
  hardware.set_transport_for_testing(make_fake_transport(&fake));
  fake->set_status_on_write("[STATUS] J0:0 J1:-180 J2:90 J3:0 J4:0 J5:0 CLAW:0");

  ASSERT_EQ(hardware.on_configure(lifecycle_state()), CallbackReturn::SUCCESS);
  ASSERT_EQ(hardware.on_activate(lifecycle_state()), CallbackReturn::SUCCESS);

  auto commands = hardware.export_command_interfaces();
  ASSERT_TRUE(commands[0].set_value(0.1));
  ASSERT_TRUE(commands[1].set_value(0.2));
  ASSERT_TRUE(commands[2].set_value(0.3));
  ASSERT_TRUE(commands[3].set_value(0.4));
  ASSERT_TRUE(commands[4].set_value(0.5));
  ASSERT_TRUE(commands[5].set_value(0.6));
  ASSERT_TRUE(commands[6].set_value(0.017));

  ASSERT_EQ(hardware.write(test_time(), test_period()), hardware_interface::return_type::OK);
  const auto writes = fake->writes();
  ASSERT_EQ(writes.size(), 2u);
  EXPECT_NE(writes.back().find("[CMD][36]"), std::string::npos);
  EXPECT_EQ(writes.back().find('\n'), writes.back().size() - 1);

  ASSERT_EQ(hardware.on_deactivate(lifecycle_state()), CallbackReturn::SUCCESS);
  EXPECT_EQ(hardware.command_positions_for_testing(), hardware.state_positions_for_testing());

  ASSERT_EQ(hardware.on_cleanup(lifecycle_state()), CallbackReturn::SUCCESS);
}

}  // namespace zyarm_hardware_interface
