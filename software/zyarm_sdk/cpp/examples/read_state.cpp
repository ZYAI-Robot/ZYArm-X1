#include <iostream>

#include "zyarm_sdk/arm.hpp"

int main()
{
  zyarm_sdk::ZyArmConfig config;
  config.port = "/dev/ttyUSB0";
  zyarm_sdk::ZyArm arm(config);
  arm.connect();
  auto state = arm.query_state(std::chrono::milliseconds(1000));
  if (!state.has_value()) {
    std::cout << "No fresh state received\n";
    return 1;
  }
  for (double value : state->positions) {
    std::cout << value << " ";
  }
  std::cout << "\n";
  arm.close();
  return 0;
}
