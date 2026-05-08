#include <iostream>
#include <memory>

#include "zyarm_sdk/teleop.hpp"

int main()
{
  zyarm_sdk::ZyArmConfig leader_config;
  leader_config.port = "/dev/ttyUSB0";
  zyarm_sdk::ZyArmConfig follower_config;
  follower_config.port = "/dev/ttyUSB1";

  auto leader = std::make_shared<zyarm_sdk::ZyArm>(leader_config);
  auto follower = std::make_shared<zyarm_sdk::ZyArm>(follower_config);
  zyarm_sdk::ZyArmTeleopPair pair(leader, follower);
  pair.connect();
  pair.start_step_mode();
  auto result = pair.step(true, false);
  if (result.has_value()) {
    std::cout << "teleop action gripper=" << result->action.positions[6] << "\n";
  }
  pair.close();
  return 0;
}
