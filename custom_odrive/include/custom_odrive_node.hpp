#ifndef CUSTOM_ODRIVE_NODE_HPP
#define CUSTOM_ODRIVE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include "custom_odrive/msg/o_drive_status.hpp"
#include "custom_odrive/msg/controller_status.hpp"
#include "custom_odrive/msg/control_message.hpp"
#include "custom_odrive/srv/axis_state.hpp"
#include "custom_odrive/srv/get_errors.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "socket_can.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <linux/can.h>
#include <linux/can/raw.h>

using std::placeholders::_1;
using std::placeholders::_2;

using ODriveStatus = custom_odrive::msg::ODriveStatus;
using ControllerStatus = custom_odrive::msg::ControllerStatus;
using ControlMessage = custom_odrive::msg::ControlMessage;

using AxisState = custom_odrive::srv::AxisState;
using GetErrors = custom_odrive::srv::GetErrors;
using Empty = std_srvs::srv::Empty;
using SetBool = std_srvs::srv::SetBool;
using Bool = std_msgs::msg::Bool;

class CustomODriveNode : public rclcpp::Node {
public:
  CustomODriveNode(const std::string& node_name);
  bool init(EpollEventLoop* event_loop);
  void deinit();

private:
  void recv_callback(const can_frame& frame);
  void subscriber_callback(const ControlMessage::SharedPtr msg);
  void drivestop_callback(const Bool::SharedPtr msg);
  void service_callback(const std::shared_ptr<AxisState::Request> request,
                        std::shared_ptr<AxisState::Response> response);
  void service_clear_errors_callback(const std::shared_ptr<Empty::Request> request,
                                     std::shared_ptr<Empty::Response> response);
  void service_set_enabled_callback(const std::shared_ptr<SetBool::Request> request,
                                    std::shared_ptr<SetBool::Response> response);
  void service_get_errors_callback(const std::shared_ptr<GetErrors::Request> request,
                                   std::shared_ptr<GetErrors::Response> response);
  void request_state_callback();
  void request_clear_errors_callback();
  void ctrl_msg_callback();
  void send_axis_idle();
  void request_idle_on_can();
  bool commands_allowed() const;
  void publish_controller_status();
  void fill_axis_state_response(std::shared_ptr<AxisState::Response> response, bool success, bool timed_out);
  inline bool verify_length(const std::string& name, uint8_t expected, uint8_t length);

  uint16_t node_id_;
  bool axis_idle_on_shutdown_;
  bool axis_idle_on_startup_{true};
  bool control_message_in_radians_{false};
  bool invert_direction_{false};
  double request_axis_state_timeout_s_{5.0};
  SocketCanIntf can_intf_ = SocketCanIntf();

  // Separate groups so control_message can be processed while request_axis_state
  // blocks (>=1s). With a single MutuallyExclusive group + SingleThreadedExecutor,
  // setpoint forwarding stops during that wait and a 1s watchdog disarms the axis.
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

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
  rclcpp::Subscription<Bool>::SharedPtr drivestop_subscriber_;

  EpollEvent srv_evt_;
  uint32_t axis_state_;
  std::mutex axis_state_mutex_;
  std::condition_variable fresh_heartbeat_;
  rclcpp::Service<AxisState>::SharedPtr service_;

  EpollEvent srv_clear_errors_evt_;
  rclcpp::Service<Empty>::SharedPtr service_clear_errors_;

  // Local enable latch (set_enabled). Independent of /drivestop.
  std::atomic<bool> enabled_{true};
  // Global drive allow from /drivestop: true = commands allowed, false = IDLE + block.
  std::atomic<bool> drive_allowed_{true};
  rclcpp::Service<SetBool>::SharedPtr service_set_enabled_;

  rclcpp::Service<GetErrors>::SharedPtr service_get_errors_;
};

#endif  // CUSTOM_ODRIVE_NODE_HPP
