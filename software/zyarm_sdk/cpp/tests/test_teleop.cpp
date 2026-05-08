#include <cassert>
#include <chrono>
#include <memory>
#include <thread>

#include "fake_transport.hpp"
#include "zyarm_sdk/teleop.hpp"

namespace
{
void wait_until_written(const std::shared_ptr<FakeTransport> & transport, std::size_t count)
{
  const auto deadline = zyarm_sdk::Clock::now() + std::chrono::seconds(1);
  while (zyarm_sdk::Clock::now() < deadline) {
    if (transport->written_line_count() >= count) {
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  assert(transport->written_line_count() >= count);
}

}  // namespace

int main()
{
  using namespace zyarm_sdk;
  ZyArmConfig leader_config;
  leader_config.port = "leader";
  ZyArmConfig follower_config;
  follower_config.port = "follower";

  auto leader_transport = std::make_shared<FakeTransport>();
  auto follower_transport = std::make_shared<FakeTransport>();
  auto leader = std::make_shared<ZyArm>(leader_config, leader_transport);
  auto follower = std::make_shared<ZyArm>(follower_config, follower_transport);
  leader->connect();
  follower->connect();

  leader_transport->feed_line("[MD][4][0 -180 90 0 0 0 50]");
  ZyArmTeleopPair pair(leader, follower);
  auto result = pair.step(false, false);
  assert(result.has_value());
  assert(result->action.positions[6] == 0.5);
  assert(follower_transport->written_lines.back().find("[CMD][36]") == 0);

  pair.start_auto_follow();
  assert(follower_transport->written_lines[1] == "[CMD][34][0.150]\n");
  assert(follower_transport->written_lines[2] == "[CMD][32][2 50]\n");
  assert(leader_transport->written_lines.back() == "[CMD][32][1 50]\n");
  leader_transport->feed_line("[MD][5][0 -180 90 0 0 0 60]");
  wait_until_written(follower_transport, 4);

  leader_transport->feed_line("[MD][7][0 -180 90 0 0 0 70]");
  wait_until_written(follower_transport, 5);
  pair.stop();
  assert(leader_transport->written_lines.back() == "[CMD][33]\n");
  assert(follower_transport->written_lines.back() == "[CMD][33]\n");
  return 0;
}
