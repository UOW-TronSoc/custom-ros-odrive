#ifndef CUSTOM_ODRIVE_NODE_HPP
#define CUSTOM_ODRIVE_NODE_HPP

/*
 * CustomODriveNode — one ROS 2 node per motor, talking ODrive CAN Simple over SocketCAN.
 *
 * Architecture (matches upstream odrive_can / ros_odrive pattern):
 *   - ROS callbacks run on a MultiThreadedExecutor (see main.cpp).
 *   - Actual CAN send/recv runs on a dedicated EpollEventLoop thread.
 *   - ROS → CAN handoff uses eventfd-backed EpollEvent objects (sub_evt_, srv_evt_, …)
 *     so the ROS thread never blocks in write() on the socket under normal paths;
 *     it sets an event, and the CAN thread performs the frame TX.
 *
 * Safety latches (both must allow motion):
 *   enabled_          — local set_enabled service (per motor)
 *   drivestop_active_ — absolute /drivestop topic (global; default OFF until a msg arrives)
 *
 * There is NO periodic keepalive. Setpoint frames are sent once per control_message.
 * If the ODrive axis watchdog is enabled, the publisher must keep streaming (~5–10×
 * watchdog rate) or the drive will disarm to IDLE.
 */

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
  explicit CustomODriveNode(const std::string& node_name);

  // Bind SocketCAN + epoll events to the shared CAN thread's event loop.
  // Must be called before spinning; returns false if the interface cannot open.
  bool init(EpollEventLoop* event_loop);

  // Optional IDLE TX (axis_idle_on_shutdown), then tear down events + socket.
  void deinit();

private:
  // --- CAN RX (called on the epoll thread) ---
  void recv_callback(const can_frame& frame);

  // --- ROS callbacks (executor threads) ---
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

  // --- EpollEvent handlers (CAN thread): perform the actual SocketCAN TX ---
  void request_state_callback();       // ClearErrors (if needed) + Set_Axis_State
  void request_clear_errors_callback();  // ClearErrors only
  void ctrl_msg_callback();            // Set_Controller_Mode (if changed) + setpoint

  // --- Helpers ---
  void send_clear_errors();            // Immediate ClearErrors CAN frame on this thread
  void send_axis_idle();               // Immediate Set_Axis_State IDLE on this thread
  void request_idle_on_can();          // Queue IDLE via srv_evt_ (cross-thread safe)
  bool commands_allowed() const;       // enabled_ && !drivestop_active_
  void publish_controller_status();    // Apply invert / radian scaling, then publish
  void fill_axis_state_response(std::shared_ptr<AxisState::Response> response, bool success,
                                bool timed_out);
  inline bool verify_length(const std::string& name, uint8_t expected, uint8_t length);

  // Parameters (filled in init())
  uint16_t node_id_;                   // ODrive CAN node_id (arbitration id = node_id<<5 | cmd)
  bool axis_idle_on_shutdown_;
  bool axis_idle_on_startup_{true};
  bool control_message_in_radians_{false};  // true: ROS pos/vel in rad(/s); CAN always turns
  bool invert_direction_{false};            // flip command + feedback sign (e.g. left wheels)
  double request_axis_state_timeout_s_{5.0};
  SocketCanIntf can_intf_ = SocketCanIntf();

  // Separate groups so control_message can be processed while request_axis_state
  // waits for confirmation. Otherwise setpoint forwarding could stop during a
  // long transition and allow the ODrive watchdog to disarm the axis.
  rclcpp::CallbackGroup::SharedPtr sub_cb_group_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;

  // ControllerStatus assembly: bit flags wait for a coherent set of cyclic msgs
  // before publishing (see recv_callback).
  short int ctrl_pub_flag_ = 0;
  std::mutex ctrl_stat_mutex_;
  uint64_t heartbeat_sequence_{0};
  ControllerStatus ctrl_stat_ = ControllerStatus();
  rclcpp::Publisher<ControllerStatus>::SharedPtr ctrl_publisher_;

  short int odrv_pub_flag_ = 0;
  std::mutex odrv_stat_mutex_;
  ODriveStatus odrv_stat_ = ODriveStatus();
  rclcpp::Publisher<ODriveStatus>::SharedPtr odrv_publisher_;

  // control_message path: ROS copies into ctrl_msg_, sets sub_evt_; CAN thread TX
  EpollEvent sub_evt_;
  std::mutex ctrl_msg_mutex_;
  ControlMessage ctrl_msg_ = ControlMessage();
  bool controller_mode_sent_ = false;  // only TX Set_Controller_Mode when mode changes
  uint32_t last_control_mode_ = 0;
  uint32_t last_input_mode_ = 0;
  rclcpp::Subscription<ControlMessage>::SharedPtr subscriber_;
  rclcpp::Subscription<Bool>::SharedPtr drivestop_subscriber_;

  // request_axis_state path: service sets axis_state_ + srv_evt_; waits on heartbeat CV
  EpollEvent srv_evt_;
  uint32_t axis_state_;
  std::mutex axis_state_mutex_;
  std::condition_variable fresh_heartbeat_;  // notified on each Heartbeat RX
  rclcpp::Service<AxisState>::SharedPtr service_;

  EpollEvent srv_clear_errors_evt_;
  rclcpp::Service<Empty>::SharedPtr service_clear_errors_;

  // Local enable latch (set_enabled). Independent of /drivestop.
  std::atomic<bool> enabled_{true};
  // /drivestop latch: true = IDLE + block motion commands; false = allow.
  // Local default is OFF (false) until a /drivestop message is received.
  std::atomic<bool> drivestop_active_{false};
  rclcpp::Service<SetBool>::SharedPtr service_set_enabled_;

  rclcpp::Service<GetErrors>::SharedPtr service_get_errors_;
};

#endif  // CUSTOM_ODRIVE_NODE_HPP
