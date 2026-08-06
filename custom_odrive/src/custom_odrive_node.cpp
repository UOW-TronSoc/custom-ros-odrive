#include "custom_odrive_node.hpp"

/*
 * Implementation notes
 * --------------------
 * CAN arbitration ID (11-bit Simple): (node_id << 5) | cmd_id
 *   cmd_id is the low 5 bits (see CmdId / ODrive CAN protocol docs).
 *
 * Feedback publish gating (ctrl_pub_flag_ / odrv_pub_flag_):
 *   ODrive sends cyclic messages independently. We OR bits as each arrives and
 *   publish when a useful combination is present (full set, or encoder-only /
 *   heartbeat+encoder) so RViz/controllers still get motion feedback even if
 *   optional cyclic msgs (Iq, torques) are disabled in firmware.
 *
 * Unit convention:
 *   ODrive CAN Simple uses turns and turns/s. With control_message_in_radians
 *   true (package default), ROS topics use rad / rad/s; we convert on the way
 *   in (commands) and out (estimates).
 */

#include "odrive_enums.h"
#include "odrive_error_decoder.hpp"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

namespace {
constexpr float kTwoPi = 6.28318530717958647692F;
}  // namespace

// Subset of ODrive CAN Simple command IDs used by this node.
enum CmdId : uint32_t {
  kHeartbeat = 0x001,
  kGetError = 0x003,
  kSetAxisState = 0x007,
  kGetEncoderEstimates = 0x009,
  kSetControllerMode = 0x00b,
  kSetInputPos,   // 0x00c
  kSetInputVel,   // 0x00d
  kSetInputTorque,  // 0x00e
  kGetIq = 0x014,
  kGetTemp,       // 0x015
  kGetBusVoltageCurrent = 0x017,
  kClearErrors = 0x018,
  kGetTorques = 0x01c,
};

enum ControlMode : uint64_t {
  kVoltageControl,   // 0 — not supported for TX here
  kTorqueControl,    // 1
  kVelocityControl,  // 2
  kPositionControl,  // 3
};

CustomODriveNode::CustomODriveNode(const std::string& node_name) : rclcpp::Node(node_name) {
  // Declarations only — values applied in init() after the process is ready.
  rclcpp::Node::declare_parameter<std::string>("interface", "can0");
  rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);
  rclcpp::Node::declare_parameter<bool>("axis_idle_on_shutdown", true);
  rclcpp::Node::declare_parameter<bool>("axis_idle_on_startup", true);
  rclcpp::Node::declare_parameter<bool>("control_message_in_radians", true);
  rclcpp::Node::declare_parameter<bool>("invert_direction", false);
  rclcpp::Node::declare_parameter<double>("request_axis_state_timeout_s", 5.0);
  rclcpp::Node::declare_parameter<bool>("start_enabled", true);

  // Two mutually exclusive groups + MultiThreadedExecutor (main.cpp) so a
  // blocking request_axis_state does not starve control_message callbacks.
  sub_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_cb_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Feedback is best-effort / KeepLast — matches typical high-rate telemetry.
  rclcpp::QoS ctrl_stat_qos(rclcpp::KeepLast(10));
  ctrl_stat_qos.best_effort();
  ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);

  rclcpp::QoS odrv_stat_qos(rclcpp::KeepLast(10));
  odrv_stat_qos.best_effort();
  odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

  // KeepLast(1): only the latest setpoint matters; publisher QoS must be compatible.
  rclcpp::QoS ctrl_msg_qos(rclcpp::KeepLast(1));
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = sub_cb_group_;
  subscriber_ = rclcpp::Node::create_subscription<ControlMessage>(
      "control_message", ctrl_msg_qos, std::bind(&CustomODriveNode::subscriber_callback, this, _1), sub_opts);

  // Absolute /drivestop (std_msgs/Bool). Local default: OFF (allow) until a message arrives.
  // true = IDLE + block motion commands; false = allow. QoS matches a system latch publisher.
  rclcpp::QoS drivestop_qos(rclcpp::KeepLast(1));
  drivestop_qos.reliable();
  drivestop_qos.transient_local();
  drivestop_subscriber_ = rclcpp::Node::create_subscription<Bool>(
      "/drivestop", drivestop_qos, std::bind(&CustomODriveNode::drivestop_callback, this, _1), sub_opts);

  rclcpp::QoS srv_qos(rclcpp::KeepAll{});

#if RCLCPP_VERSION_MAJOR >= 18
  auto srv_qos_profile = srv_qos;
#else
  auto srv_qos_profile = srv_qos.get_rmw_qos_profile();
#endif

  service_ = rclcpp::Node::create_service<AxisState>(
      "request_axis_state", std::bind(&CustomODriveNode::service_callback, this, _1, _2), srv_qos_profile,
      srv_cb_group_);
  service_clear_errors_ = rclcpp::Node::create_service<Empty>(
      "clear_errors", std::bind(&CustomODriveNode::service_clear_errors_callback, this, _1, _2), srv_qos_profile,
      srv_cb_group_);
  service_set_enabled_ = rclcpp::Node::create_service<SetBool>(
      "set_enabled", std::bind(&CustomODriveNode::service_set_enabled_callback, this, _1, _2), srv_qos_profile,
      srv_cb_group_);
  service_get_errors_ = rclcpp::Node::create_service<GetErrors>(
      "get_errors", std::bind(&CustomODriveNode::service_get_errors_callback, this, _1, _2), srv_qos_profile,
      srv_cb_group_);
}

void CustomODriveNode::send_clear_errors() {
  struct can_frame frame = {};
  frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
  write_le<uint8_t>(0, frame.data);  // Identify=0 (no LED blink)
  frame.can_dlc = 1;
  can_intf_.send_can_frame(frame);
}

void CustomODriveNode::send_axis_idle() {
  // Direct TX (startup / shutdown paths already on a context that can write).
  struct can_frame frame = {};
  frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
  write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
  frame.can_dlc = 4;
  can_intf_.send_can_frame(frame);
}

void CustomODriveNode::request_idle_on_can() {
  // Cross-thread: stash desired state and wake the CAN thread via eventfd.
  {
    std::unique_lock<std::mutex> guard(axis_state_mutex_);
    axis_state_ = ODriveAxisState::AXIS_STATE_IDLE;
  }
  srv_evt_.set();
}

bool CustomODriveNode::commands_allowed() const {
  return enabled_.load() && !drivestop_active_.load();
}

void CustomODriveNode::deinit() {
  if (axis_idle_on_shutdown_) {
    send_axis_idle();
  }

  sub_evt_.deinit();
  srv_evt_.deinit();
  srv_clear_errors_evt_.deinit();
  can_intf_.deinit();
}

bool CustomODriveNode::init(EpollEventLoop* event_loop) {
  node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
  axis_idle_on_shutdown_ = rclcpp::Node::get_parameter("axis_idle_on_shutdown").as_bool();
  axis_idle_on_startup_ = rclcpp::Node::get_parameter("axis_idle_on_startup").as_bool();
  control_message_in_radians_ = rclcpp::Node::get_parameter("control_message_in_radians").as_bool();
  invert_direction_ = rclcpp::Node::get_parameter("invert_direction").as_bool();
  request_axis_state_timeout_s_ = rclcpp::Node::get_parameter("request_axis_state_timeout_s").as_double();
  enabled_.store(rclcpp::Node::get_parameter("start_enabled").as_bool());
  std::string interface = rclcpp::Node::get_parameter("interface").as_string();

  // Service always waits at least 1s (upstream behavior); clamp timeout accordingly.
  if (request_axis_state_timeout_s_ < 1.0) {
    RCLCPP_WARN(rclcpp::Node::get_logger(),
                "request_axis_state_timeout_s (%.3f) is below the 1s minimum wait; clamping to 1.0",
                request_axis_state_timeout_s_);
    request_axis_state_timeout_s_ = 1.0;
  }

  if (!can_intf_.init(interface, event_loop, std::bind(&CustomODriveNode::recv_callback, this, _1))) {
    RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
    return false;
  }
  if (!sub_evt_.init(event_loop, std::bind(&CustomODriveNode::ctrl_msg_callback, this))) {
    RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize subscriber event");
    return false;
  }
  if (!srv_evt_.init(event_loop, std::bind(&CustomODriveNode::request_state_callback, this))) {
    RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize service event");
    return false;
  }
  if (!srv_clear_errors_evt_.init(event_loop, std::bind(&CustomODriveNode::request_clear_errors_callback, this))) {
    RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize clear errors service event");
    return false;
  }

  RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
  RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
  RCLCPP_INFO(rclcpp::Node::get_logger(), "control_message_in_radians: %s",
              control_message_in_radians_ ? "true" : "false");
  RCLCPP_INFO(rclcpp::Node::get_logger(), "invert_direction: %s", invert_direction_ ? "true" : "false");
  RCLCPP_INFO(rclcpp::Node::get_logger(), "request_axis_state_timeout_s: %.3f", request_axis_state_timeout_s_);
  RCLCPP_INFO(rclcpp::Node::get_logger(), "start_enabled: %s", enabled_.load() ? "true" : "false");
  RCLCPP_INFO(rclcpp::Node::get_logger(), "axis_idle_on_startup: %s", axis_idle_on_startup_ ? "true" : "false");
  RCLCPP_INFO(rclcpp::Node::get_logger(),
              "listening on /drivestop (true=stop, false=allow); local default OFF (allow)");

  if (axis_idle_on_startup_) {
    RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting ClearErrors + IDLE on startup");
    send_clear_errors();
    send_axis_idle();
  }

  return true;
}

void CustomODriveNode::fill_axis_state_response(std::shared_ptr<AxisState::Response> response, bool success,
                                               bool timed_out) {
  // Snapshot latest heartbeat-derived fields under ctrl_stat_mutex_ (caller holds it
  // for the wait path; rejection paths lock before calling).
  response->success = success;
  response->timed_out = timed_out;
  response->axis_state = ctrl_stat_.axis_state;
  response->active_errors = ctrl_stat_.active_errors;
  response->procedure_result = ctrl_stat_.procedure_result;
}

void CustomODriveNode::publish_controller_status() {
  ControllerStatus msg = ctrl_stat_;
  const float sign = invert_direction_ ? -1.0F : 1.0F;

  // Invert applies to mechanical quantities so left/right wheels share a chassis frame.
  msg.pos_estimate *= sign;
  msg.vel_estimate *= sign;
  msg.torque_target *= sign;
  msg.torque_estimate *= sign;
  msg.enabled = enabled_.load();

  if (control_message_in_radians_) {
    // CAN estimates are turns / turns/s → rad / rad/s for ROS.
    msg.pos_estimate *= kTwoPi;
    msg.vel_estimate *= kTwoPi;
  }

  ctrl_publisher_->publish(msg);
}

void CustomODriveNode::recv_callback(const can_frame& frame) {
  // Ignore frames for other node_ids on a shared bus.
  if (((frame.can_id >> 5) & 0x3F) != node_id_) return;

  switch (frame.can_id & 0x1F) {
    case CmdId::kHeartbeat: {
      if (!verify_length("kHeartbeat", 8, frame.can_dlc)) break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      ctrl_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
      ctrl_stat_.axis_state = read_le<uint8_t>(frame.data + 4);
      ctrl_stat_.procedure_result = read_le<uint8_t>(frame.data + 5);
      ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
      ctrl_pub_flag_ |= 0b0001;
      // Wake request_axis_state waiters so they can re-check procedure_result.
      fresh_heartbeat_.notify_one();
      break;
    }
    case CmdId::kGetError: {
      if (!verify_length("kGetError", 8, frame.can_dlc)) break;
      std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
      odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
      odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
      odrv_pub_flag_ |= 0b001;
      break;
    }
    case CmdId::kGetEncoderEstimates: {
      if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc)) break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
      ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
      ctrl_pub_flag_ |= 0b0010;
      break;
    }
    case CmdId::kGetIq: {
      if (!verify_length("kGetIq", 8, frame.can_dlc)) break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
      ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
      ctrl_pub_flag_ |= 0b0100;
      break;
    }
    case CmdId::kGetTemp: {
      if (!verify_length("kGetTemp", 8, frame.can_dlc)) break;
      std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
      odrv_stat_.fet_temperature = read_le<float>(frame.data + 0);
      odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
      odrv_pub_flag_ |= 0b010;
      break;
    }
    case CmdId::kGetBusVoltageCurrent: {
      if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc)) break;
      std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
      odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
      odrv_stat_.bus_current = read_le<float>(frame.data + 4);
      odrv_pub_flag_ |= 0b100;
      break;
    }
    case CmdId::kGetTorques: {
      if (!verify_length("kGetTorques", 8, frame.can_dlc)) break;
      std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
      ctrl_stat_.torque_target = read_le<float>(frame.data + 0);
      ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
      ctrl_pub_flag_ |= 0b1000;
      break;
    }
    // Host→ODrive commands: no payload handling if somehow received.
    case CmdId::kSetAxisState:
    case CmdId::kSetControllerMode:
    case CmdId::kSetInputPos:
    case CmdId::kSetInputVel:
    case CmdId::kSetInputTorque:
    case CmdId::kClearErrors: {
      break;
    }
    default: {
      RCLCPP_WARN(rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
      break;
    }
  }

  // Publish controller_status when we have a full cyclic set (0b1111), encoder-only
  // (0b0010), or heartbeat+encoder (0b0011). Resets flags after publish.
  if (ctrl_pub_flag_ == 0b1111 || ctrl_pub_flag_ == 0b0010 || ctrl_pub_flag_ == 0b0011) {
    std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
    publish_controller_status();
    ctrl_pub_flag_ = 0;
  }

  // odrive_status needs error + temp + bus (all three cyclic messages enabled).
  if (odrv_pub_flag_ == 0b111) {
    odrv_publisher_->publish(odrv_stat_);
    odrv_pub_flag_ = 0;
  }
}

void CustomODriveNode::subscriber_callback(const ControlMessage::SharedPtr msg) {
  // Drop silently while disabled or drivestop asserted — do not feed the watchdog.
  if (!commands_allowed()) {
    return;
  }
  std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
  ctrl_msg_ = *msg;
  sub_evt_.set();  // Wake CAN thread → ctrl_msg_callback()
}

void CustomODriveNode::drivestop_callback(const Bool::SharedPtr msg) {
  // true = stop (IDLE + block); false = drivestop off (allow commands again).
  if (!msg->data) {
    drivestop_active_.store(false);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "/drivestop=false: drivestop off — commands allowed");
    return;
  }

  // Idempotent: only queue one IDLE when transitioning to asserted.
  const bool already_stopped = drivestop_active_.exchange(true);
  if (already_stopped) {
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "/drivestop=true: already asserted");
    return;
  }

  request_idle_on_can();
  RCLCPP_WARN(rclcpp::Node::get_logger(),
              "/drivestop=true: IDLE requested; motion commands blocked until /drivestop=false");
}

void CustomODriveNode::service_callback(const std::shared_ptr<AxisState::Request> request,
                                        std::shared_ptr<AxisState::Response> response) {
  const bool requesting_idle = request->axis_requested_state == ODriveAxisState::AXIS_STATE_IDLE;

  // Always allow IDLE even when stopped/disabled so the axis can be put safe.
  if (drivestop_active_.load() && !requesting_idle) {
    RCLCPP_WARN(rclcpp::Node::get_logger(),
                "rejecting axis state %u: /drivestop is true (publish /drivestop data:=false first)",
                request->axis_requested_state);
    std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
    fill_axis_state_response(response, false, false);
    return;
  }

  if (!enabled_.load() && !requesting_idle) {
    RCLCPP_WARN(rclcpp::Node::get_logger(),
                "rejecting axis state %u: motor is disabled (call set_enabled with data:=true)",
                request->axis_requested_state);
    std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
    fill_axis_state_response(response, false, false);
    return;
  }

  {
    std::unique_lock<std::mutex> guard(axis_state_mutex_);
    axis_state_ = request->axis_requested_state;
    RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting axis state: %d", axis_state_);
  }
  srv_evt_.set();

  // Block until the heartbeats show the requested axis state has actually been reached
  // (and the procedure is no longer BUSY), or until the timeout expires. We still
  // preserve the minimum 1s wait to avoid reporting success on an immediate reply.
  std::unique_lock<std::mutex> guard(ctrl_stat_mutex_);
  auto call_time = std::chrono::steady_clock::now();
  const auto timeout = std::chrono::duration<double>(request_axis_state_timeout_s_);

  const bool completed = fresh_heartbeat_.wait_for(guard, timeout, [this, &call_time, &request]() {
    const bool state_matches = this->ctrl_stat_.axis_state == request->axis_requested_state;
    const bool is_busy = this->ctrl_stat_.procedure_result == ODriveProcedureResult::PROCEDURE_RESULT_BUSY;
    const bool minimum_time_passed = (std::chrono::steady_clock::now() - call_time >= std::chrono::seconds(1));
    const bool procedure_finished = !is_busy;
    return state_matches && minimum_time_passed && procedure_finished;
  });

  if (!completed) {
    RCLCPP_ERROR(rclcpp::Node::get_logger(),
                 "request_axis_state timed out after %.3f s (no completing heartbeat/procedure)",
                 request_axis_state_timeout_s_);
    fill_axis_state_response(response, false, true);
    return;
  }

  fill_axis_state_response(response, true, false);
}

void CustomODriveNode::service_clear_errors_callback(const std::shared_ptr<Empty::Request> /*request*/,
                                                     std::shared_ptr<Empty::Response> /*response*/) {
  RCLCPP_INFO(rclcpp::Node::get_logger(), "clearing errors");
  srv_clear_errors_evt_.set();
}

void CustomODriveNode::service_set_enabled_callback(const std::shared_ptr<SetBool::Request> request,
                                                    std::shared_ptr<SetBool::Response> response) {
  if (request->data) {
    // Cannot arm while global stop is latched.
    if (drivestop_active_.load()) {
      response->success = false;
      response->message = "cannot enable: /drivestop is true";
      RCLCPP_WARN(rclcpp::Node::get_logger(), "set_enabled(true) rejected: /drivestop is true");
      return;
    }
    enabled_.store(true);
    response->success = true;
    response->message = "motor enabled";
    RCLCPP_INFO(rclcpp::Node::get_logger(), "motor enabled");
    return;
  }

  // Disable: ignore further control_message and request IDLE on the bus.
  enabled_.store(false);
  request_idle_on_can();
  response->success = true;
  response->message = "motor disabled; IDLE requested";
  RCLCPP_WARN(rclcpp::Node::get_logger(), "motor disabled; control ignored until set_enabled(true)");
}

void CustomODriveNode::service_get_errors_callback(const std::shared_ptr<GetErrors::Request> /*request*/,
                                                   std::shared_ptr<GetErrors::Response> response) {
  // Snapshot last Get_Error cyclic payload and decode bitfields to strings.
  uint32_t active_errors;
  uint32_t disarm_reason;
  {
    std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
    active_errors = odrv_stat_.active_errors;
    disarm_reason = odrv_stat_.disarm_reason;
  }

  response->active_errors = active_errors;
  response->disarm_reason = disarm_reason;
  response->active_errors_decoded = odrive_decode::decode_odrive_error(active_errors);
  response->disarm_reason_decoded = odrive_decode::decode_odrive_error(disarm_reason);
}

void CustomODriveNode::request_state_callback() {
  // CAN thread: optional ClearErrors then Set_Axis_State for the pending request.
  uint32_t axis_state;
  {
    std::unique_lock<std::mutex> guard(axis_state_mutex_);
    axis_state = axis_state_;
  }

  struct can_frame frame;

  // Non-zero state requests clear errors first (axis_state_ 0 is unused as a request).
  if (axis_state != 0) {
    send_clear_errors();
  }

  frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
  write_le<uint32_t>(axis_state, frame.data);
  frame.can_dlc = 4;
  can_intf_.send_can_frame(frame);
}

void CustomODriveNode::request_clear_errors_callback() {
  send_clear_errors();
}

void CustomODriveNode::ctrl_msg_callback() {
  // CAN thread: translate latest ControlMessage into Simple setpoint frame(s).
  if (!commands_allowed()) {
    return;
  }

  ControlMessage ctrl_msg;
  {
    std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
    ctrl_msg = ctrl_msg_;
  }

  const uint32_t control_mode = ctrl_msg.control_mode;
  const uint32_t input_mode = ctrl_msg.input_mode;
  const float sign = invert_direction_ ? -1.0F : 1.0F;

  // Only send Set_Controller_Mode when mode pair changes (reduces bus load).
  if (!controller_mode_sent_ || control_mode != last_control_mode_ || input_mode != last_input_mode_) {
    struct can_frame mode_frame = {};
    mode_frame.can_id = node_id_ << 5 | kSetControllerMode;
    write_le<uint32_t>(control_mode, mode_frame.data);
    write_le<uint32_t>(input_mode, mode_frame.data + 4);
    mode_frame.can_dlc = 8;
    can_intf_.send_can_frame(mode_frame);

    controller_mode_sent_ = true;
    last_control_mode_ = control_mode;
    last_input_mode_ = input_mode;
  }

  struct can_frame frame = {};
  switch (control_mode) {
    case ControlMode::kVoltageControl: {
      RCLCPP_ERROR(rclcpp::Node::get_logger(), "Voltage Control Mode (0) is not currently supported");
      return;
    }
    case ControlMode::kTorqueControl: {
      RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_torque");
      frame.can_id = node_id_ << 5 | kSetInputTorque;
      write_le<float>(ctrl_msg.input_torque * sign, frame.data);
      frame.can_dlc = 4;
      break;
    }
    case ControlMode::kVelocityControl: {
      RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_vel");
      frame.can_id = node_id_ << 5 | kSetInputVel;
      float input_vel = ctrl_msg.input_vel * sign;
      if (control_message_in_radians_) {
        input_vel /= kTwoPi;  // rad/s → turns/s
      }
      write_le<float>(input_vel, frame.data);
      write_le<float>(ctrl_msg.input_torque * sign, frame.data + 4);  // torque FF
      frame.can_dlc = 8;
      break;
    }
    case ControlMode::kPositionControl: {
      RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_pos");
      frame.can_id = node_id_ << 5 | kSetInputPos;
      float input_pos = ctrl_msg.input_pos * sign;
      float input_vel = ctrl_msg.input_vel * sign;
      if (control_message_in_radians_) {
        input_pos /= kTwoPi;
        input_vel /= kTwoPi;
      }
      // Protocol: vel/torque feedforward are int16 in milli-units.
      write_le<float>(input_pos, frame.data);
      write_le<int16_t>(static_cast<int16_t>(input_vel * 1000.0F), frame.data + 4);
      write_le<int16_t>(static_cast<int16_t>(ctrl_msg.input_torque * sign * 1000.0F), frame.data + 6);
      frame.can_dlc = 8;
      break;
    }
    default:
      RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported control_mode: %d", control_mode);
      return;
  }

  // This TX also resets the ODrive axis watchdog when watchdog is enabled.
  can_intf_.send_can_frame(frame);
}

inline bool CustomODriveNode::verify_length(const std::string& name, uint8_t expected, uint8_t length) {
  bool valid = expected == length;
  RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
  if (!valid)
    RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
  return valid;
}
