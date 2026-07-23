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

  // Multi-threaded so control_message keeps forwarding to CAN while
  // request_axis_state blocks (>=1s). Required with a 1s ODrive watchdog.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  event_loop.request_stop();
  if (can_event_loop.joinable()) can_event_loop.join();

  node->deinit();
  rclcpp::shutdown();
  return 0;
}
