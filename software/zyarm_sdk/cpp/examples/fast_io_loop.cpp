#include <chrono>
#include <thread>

#include "zyarm_sdk/arm.hpp"

int main()
{
  zyarm_sdk::ZyArmConfig config;
  config.port = "/dev/ttyUSB0";
  zyarm_sdk::ZyArm arm(config);
  arm.connect();
  zyarm_sdk::JointArray target{0, 0, 0, 0, 0, 0, 0.5};
  for (int index = 0; index < 100; ++index) {
    arm.fast_io(target);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  arm.close();
  return 0;
}
