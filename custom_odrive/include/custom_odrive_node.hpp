#ifndef CUSTOM_ODRIVE_NODE_HPP
#define CUSTOM_ODRIVE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include "custom_odrive/msg/o_drive_status.hpp"
#include "custom_odrive/msg/controller_status.hpp"
#include "custom_odrive/msg/control_message.hpp"
#include "custom_odrive/srv/axis_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "socket_can.hpp"

#include <mutex>
#include <condition_variable>
#include <linux/can.h>
#include <linux/can/raw.h>

using std::placeholders::_1;
using std::placeholders::_2;

using ODriveStatus = custom_odrive::msg::ODriveStatus;
using ControllerStatus = custom_odrive::msg::ControllerStatus;
using ControlMessage = custom_odrive::msg::ControlMessage;

using AxisState = custom_odrive::srv::AxisState;
using Empty = std_srvs::srv::Empty;

class CustomODriveNode : public rclcpp::Node {
public:
  CustomODriveNode(const std::string& node_name);
  bool init(EpollEventLoop* event_loop);
  void deinit();

private:
  void recv_callback(const can_frame& frame);
  void subscriber_callback(const ControlMessage::SharedPtr msg);
  void service_callback(const std::shared_ptr<AxisState::Request> request,
                        std::shared_ptr<AxisState::Response> response);
  void service_clear_errors_callback(const std::shared_ptr<Empty::Request> request,
                                     std::shared_ptr<Empty::Response> response);
  void request_state_callback();
  void request_clear_errors_callback();
  void ctrl_msg_callback();
  void publish_controller_status();
  inline bool verify_length(const std::string& name, uint8_t expected, uint8_t length);

  uint16_t node_id_;
  bool axis_idle_on_shutdown_;
  bool control_message_in_radians_{false};
  bool invert_direction_{false};
  SocketCanIntf can_intf_ = SocketCanIntf();

  short int ctrl_pub_flag_ = 0;
  std::mutex ctrl_stat_mutex_;
  ControllerStatus ctrl_stat_ = ControllerStatus();
  rclcpp::Publisher<ControllerStatus>::SharedPtr ctrl_publisher_;

  short int odrv_pub_flag_ = 0;
  std::mutex odrv_stat_mutex_;
  ODriveStatus odrv_stat_ = ODriveStatus();
  rclcpp::Publisher<ODriveStatus>::SharedPtr odrv_publisher_;

  EpollEvent sub_evt_;
  std::mutex ctrl_msg_mutex_;
  ControlMessage ctrl_msg_ = ControlMessage();
  bool controller_mode_sent_ = false;
  uint32_t last_control_mode_ = 0;
  uint32_t last_input_mode_ = 0;
  rclcpp::Subscription<ControlMessage>::SharedPtr subscriber_;

  EpollEvent srv_evt_;
  uint32_t axis_state_;
  std::mutex axis_state_mutex_;
  std::condition_variable fresh_heartbeat_;
  rclcpp::Service<AxisState>::SharedPtr service_;

  EpollEvent srv_clear_errors_evt_;
  rclcpp::Service<Empty>::SharedPtr service_clear_errors_;
};

#endif  // CUSTOM_ODRIVE_NODE_HPP
