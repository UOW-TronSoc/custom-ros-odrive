#include "custom_odrive_node.hpp"
#include "epoll_event_loop.hpp"
#include "socket_can.hpp"
#include <thread>

/*
 * Process layout
 * --------------
 * 1) EpollEventLoop thread — SocketCAN RX + EpollEvent TX callbacks (non-blocking I/O).
 * 2) MultiThreadedExecutor — ROS subscriptions/services.
 *
 * MultiThreadedExecutor is required: request_axis_state blocks ≥1s waiting on
 * heartbeats. With a single thread, control_message would stall and a 1s axis
 * watchdog would disarm the motor during that wait.
 */

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  EpollEventLoop event_loop;
  auto node = std::make_shared<CustomODriveNode>("CustomODriveNode");

  if (!node->init(&event_loop)) return -1;

  // CAN / epoll thread: blocks in epoll_wait until request_stop().
  std::thread can_event_loop([&event_loop]() { event_loop.run_until_empty(); });

  // Multi-threaded so control_message keeps forwarding to CAN while
  // request_axis_state blocks (>=1s). Required with a 1s ODrive watchdog.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Clean shutdown: wake epoll, join CAN thread, optional IDLE, close socket.
  event_loop.request_stop();
  if (can_event_loop.joinable()) can_event_loop.join();

  node->deinit();
  rclcpp::shutdown();
  return 0;
}
