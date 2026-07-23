#!/usr/bin/env python3
"""Minimal closed-loop velocity test for custom_odrive.

Streams control_message continuously (feeds the axis watchdog), clears errors,
enables, requests CLOSED_LOOP, and only proceeds if axis_state actually becomes 8.

Why continuous publish matters
------------------------------
custom_odrive_node does not send a periodic keepalive. Each control_message
becomes one CAN setpoint frame, which resets the ODrive watchdog. If this
script stops publishing (or publishes too slowly), the drive disarms to IDLE.

Also: request_axis_state may report success=true for CLOSED_LOOP after ~1s even
when the axis never left IDLE (e.g. latched WATCHDOG / NOT_CALIBRATED). Always
confirm controller_status.axis_state == 8 before trusting motion.

Examples (with example_multi_launch.py running):
  ros2 run custom_odrive velocity_ramp_test -- --ns wheel_fr
  ros2 run custom_odrive velocity_ramp_test -- --ns /wheel_bl --target-vel 10
"""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, SetBool

from custom_odrive.msg import ControlMessage, ControllerStatus
from custom_odrive.srv import AxisState, GetErrors

# Match ODrive / custom_odrive enums (see odrive_enums.h / README).
CONTROL_MODE_VELOCITY = 2
INPUT_MODE_VEL_RAMP = 2
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP = 8


def normalize_ns(ns: str) -> str:
    """Accept wheel_fl, /wheel_fl, or /wheel_fl/ → /wheel_fl."""
    ns = ns.strip()
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/") or "/"


class VelocityTest(Node):
    """One-shot client: stream setpoints under a motor namespace and arm closed-loop."""

    def __init__(self, namespace: str, rate_hz: float, input_vel: float) -> None:
        super().__init__("velocity_ramp_test")
        self._ns = namespace
        self._rate_hz = rate_hz
        self._input_vel = input_vel
        self._vel = 0.0  # published each tick; set to target after CLOSED_LOOP confirmed
        self._axis_state = 0
        self._active_errors = 0

        self._pub = self.create_publisher(ControlMessage, f"{self._ns}/control_message", 10)
        # Node publishes controller_status as best_effort — must match or we get no messages
        status_qos = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            ControllerStatus, f"{self._ns}/controller_status", self._on_status, status_qos
        )
        self.create_timer(1.0 / rate_hz, self._tick)

        self._cli_enable = self.create_client(SetBool, f"{self._ns}/set_enabled")
        self._cli_state = self.create_client(AxisState, f"{self._ns}/request_axis_state")
        self._cli_clear = self.create_client(Empty, f"{self._ns}/clear_errors")
        self._cli_errors = self.create_client(GetErrors, f"{self._ns}/get_errors")

        self.get_logger().info(
            f"targeting {self._ns} @ {rate_hz:.1f} Hz, target_vel={input_vel:.3f} rad/s"
        )

    def _tick(self) -> None:
        """Publish current velocity setpoint (feeds watchdog every 1/rate_hz)."""
        msg = ControlMessage()
        msg.control_mode = CONTROL_MODE_VELOCITY
        msg.input_mode = INPUT_MODE_VEL_RAMP
        msg.input_pos = 0.0
        msg.input_vel = float(self._vel)
        msg.input_torque = 0.0
        self._pub.publish(msg)

    def _on_status(self, msg: ControllerStatus) -> None:
        self._axis_state = int(msg.axis_state)
        self._active_errors = int(msg.active_errors)

    def wait_services(self, timeout_s: float = 10.0) -> bool:
        self.get_logger().info(f"waiting for services under {self._ns} ...")
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            if (
                self._cli_enable.service_is_ready()
                and self._cli_state.service_is_ready()
                and self._cli_clear.service_is_ready()
            ):
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().error(
            f"services not available under {self._ns} — is that motor's custom_odrive_node running?"
        )
        return False

    def set_enabled(self, enabled: bool) -> bool:
        req = SetBool.Request()
        req.data = enabled
        fut = self._cli_enable.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            self.get_logger().error("set_enabled failed")
            return False
        self.get_logger().info(f"set_enabled({enabled}): {fut.result().message}")
        return fut.result().success

    def clear_errors(self) -> None:
        fut = self._cli_clear.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        self.get_logger().info("clear_errors called")

    def print_errors(self) -> None:
        if not self._cli_errors.service_is_ready():
            return
        fut = self._cli_errors.call_async(GetErrors.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None:
            return
        r = fut.result()
        self.get_logger().info(
            f"get_errors: active=0x{r.active_errors:08X} {list(r.active_errors_decoded)} | "
            f"disarm=0x{r.disarm_reason:08X} {list(r.disarm_reason_decoded)}"
        )

    def request_state(self, state: int) -> AxisState.Response | None:
        req = AxisState.Request()
        req.axis_requested_state = state
        fut = self._cli_state.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15.0)
        if not fut.done() or fut.result() is None:
            self.get_logger().error("request_axis_state failed")
            return None
        r = fut.result()
        self.get_logger().info(
            f"request_axis_state({state}): success={r.success} timed_out={r.timed_out} "
            f"axis_state={r.axis_state} procedure_result={r.procedure_result} "
            f"errors=0x{r.active_errors:08X}"
        )
        return r

    def wait_for_axis_state(self, wanted: int, timeout_s: float = 3.0) -> bool:
        """Poll controller_status until axis_state matches (or timeout)."""
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._axis_state == wanted:
                return True
        return False


def parse_args(argv: list[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Watchdog-safe velocity test for one custom_odrive motor")
    p.add_argument(
        "--ns",
        required=True,
        help="ROS namespace of the motor (e.g. wheel_fr, /wheel_bl, wheel_br)",
    )
    p.add_argument("--rate", type=float, default=10.0, help="control_message rate Hz (default: 10)")
    p.add_argument(
        "--target-vel",
        type=float,
        default=20.0,
        help="velocity setpoint in rad/s (default: 20)",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    ns = normalize_ns(args.ns)

    rclpy.init()
    node = VelocityTest(namespace=ns, rate_hz=args.rate, input_vel=args.target_vel)
    try:
        # Stream zeros BEFORE arming so the watchdog is fed the whole time —
        # otherwise CLOSED_LOOP + silence ≈ immediate WATCHDOG_TIMER_EXPIRED.
        node._vel = 0.0
        warmup = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < warmup:
            rclpy.spin_once(node, timeout_sec=0.05)

        if not node.wait_services():
            return 1

        node.clear_errors()
        time.sleep(0.2)
        node.print_errors()

        if not node.set_enabled(True):
            return 1

        resp = node.request_state(AXIS_STATE_CLOSED_LOOP)
        if resp is None:
            return 1

        # Do NOT trust success alone — CLOSED_LOOP returns success after ~1s even if still IDLE
        if not node.wait_for_axis_state(AXIS_STATE_CLOSED_LOOP, timeout_s=3.0):
            node.get_logger().error(
                f"NOT in CLOSED_LOOP (axis_state={node._axis_state}, errors=0x{node._active_errors:08X}). "
                "LED will not flash green. Common causes: WATCHDOG_TIMER_EXPIRED still latched, "
                "NOT_CALIBRATED (procedure_result=14), or other active_errors."
            )
            node.print_errors()
            return 1

        node.get_logger().info("axis_state=8 CONFIRMED — LED should flash green")
        node._vel = args.target_vel
        node.get_logger().info(
            f"publishing input_vel={args.target_vel} rad/s @ {args.rate} Hz on "
            f"{ns}/control_message — Ctrl-C to stop"
        )
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node._axis_state != AXIS_STATE_CLOSED_LOOP:
                node.get_logger().error(
                    f"dropped out of CLOSED_LOOP (axis_state={node._axis_state}, "
                    f"errors=0x{node._active_errors:08X})"
                )
                node.print_errors()
                return 1
        return 0
    except KeyboardInterrupt:
        return 130
    finally:
        # Always try to zero velocity and IDLE so we do not leave the wheel spinning.
        try:
            node.get_logger().warn("stopping: vel=0, IDLE")
            node._vel = 0.0
            end = time.monotonic() + 0.5
            while rclpy.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
            if rclpy.ok() and node._cli_state.service_is_ready():
                node.request_state(AXIS_STATE_IDLE)
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
