#include "custom_odrive_node.hpp"
#include "epoll_event_loop.hpp"
#include "socket_can.hpp"
#include <thread>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  EpollEventLoop event_loop;
  auto node = std::make_shared<CustomODriveNode>("CustomODriveNode");

  if (!node->init(&event_loop)) return -1;

  std::thread can_event_loop([&event_loop]() { event_loop.run_until_empty(); });
  rclcpp::spin(node);
  node->deinit();
  rclcpp::shutdown();
  return 0;
}
