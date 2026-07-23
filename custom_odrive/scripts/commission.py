#!/usr/bin/env python3
"""Commission an ODrive over SocketCAN via the odrive Python package (Fibre-over-CAN).

Reads SERIAL_NUMBER from a motor config .py, optionally parks a sibling
custom_odrive_node, applies the config assignments, optionally calibrates
(with an off-ground confirmation), and optionally save_configuration().

Requires: pip install --upgrade odrive  (>=0.6.11.post0) and ODrive FW >=0.6.11.

Why Fibre-over-CAN (not CAN Simple / RxSdo):
  Full nested config (motor type, watchdog, cyclic rates, …) is awkward over
  CAN Simple. odrivetool 0.6.11+ speaks the same Fibre API over SocketCAN, so
  we reuse normal `odrv.axis0.config... = ...` scripts without USB.

Why a separate script (not inside custom_odrive_node):
  Commissioning is a rare maintenance operation. Keeping it out of the C++
  realtime control loop avoids hanging the epoll CAN path on long Fibre I/O,
  calibration, or save/reboot.

Flow:
  1. Resolve SERIAL_NUMBER from --config (only identity source; no CLI override)
  2. Park sibling ROS node (--ns) or confirm bench mode
  3. Connect Fibre-over-CAN to that serial on --can
  4. exec config assignments (odrv / enums / math injected)
  5. Optional: confirm wheel off ground, then FULL_CALIBRATION_SEQUENCE
  6. Optional: save_configuration() (device reboots)

Examples:
  ros2 run custom_odrive commission -- \\
    --can can_core \\
    --config $(ros2 pkg prefix custom_odrive)/share/custom_odrive/config/wheel_fl_motor_config.py \\
    --ns /wheel_fl \\
    --calibrate \\
    --save
"""

from __future__ import annotations

import argparse
import ast
import math
import sys
import time
from pathlib import Path
from typing import Any


# ---------------------------------------------------------------------------
# Small helpers
# ---------------------------------------------------------------------------


def normalize_ns(ns: str) -> str:
    """Ensure a leading slash and no trailing slash (ROS absolute namespace).

    Accepts `wheel_fl`, `/wheel_fl`, or `/wheel_fl/` and returns `/wheel_fl`.
    """
    ns = ns.strip()
    if not ns.startswith("/"):
        ns = "/" + ns
    return ns.rstrip("/") or "/"


def prompt_yes(message: str) -> bool:
    """Interactive y/yes confirm; EOF or anything else is treated as no.

    Used for safety gates (bench-mode coexistence, wheel-off-ground before cal).
    There is intentionally no `--yes` skip flag in v1.
    """
    try:
        answer = input(message).strip().lower()
    except EOFError:
        # Non-interactive stdin (piped / CI) → refuse rather than assume yes.
        return False
    return answer in ("y", "yes")


# ---------------------------------------------------------------------------
# Motor config file handling
# ---------------------------------------------------------------------------
#
# Each wheel has a .py like:
#   SERIAL_NUMBER = "394D353B3231"          # identity for this ESC only
#   odrv.axis0.config.can.node_id = 1      # applied after Fibre connect
#   ...
#
# SERIAL_NUMBER must stay in the file (not a CLI flag) so a config cannot be
# pointed at the wrong physical ODrive by accident.
#


def read_serial_number(config_path: Path) -> str:
    """Extract top-level SERIAL_NUMBER = \"...\" without executing odrv assignments.

    Config files mix metadata (SERIAL_NUMBER) with live Fibre assignments that
    need a connected `odrv`. If we `exec` the whole file before connect, every
    `odrv.*` line would NameError. Parse with ast so we only pull the string
    constant first, then connect, then exec the assignments.
    """
    source = config_path.read_text(encoding="utf-8")
    try:
        tree = ast.parse(source, filename=str(config_path))
    except SyntaxError as exc:
        raise SystemExit(f"failed to parse config {config_path}: {exc}") from exc

    # Walk top-level statements only (not nested); SERIAL_NUMBER must be module-level.
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        for target in node.targets:
            if isinstance(target, ast.Name) and target.id == "SERIAL_NUMBER":
                try:
                    # literal_eval rejects calls / names — forces a real string constant.
                    value = ast.literal_eval(node.value)
                except (ValueError, TypeError) as exc:
                    raise SystemExit(
                        f"SERIAL_NUMBER in {config_path} must be a string literal"
                    ) from exc
                if not isinstance(value, str) or not value.strip():
                    raise SystemExit(f"SERIAL_NUMBER in {config_path} is empty")
                return value.strip()

    raise SystemExit(
        f"{config_path} must define SERIAL_NUMBER = \"...\" "
        "(only source of target ESC identity)"
    )


def build_config_globals(odrv: Any) -> dict[str, Any]:
    """Globals for exec'ing the motor config (odrv + enums + math).

    Configs are written like odrivetool snippets so they can be copy-pasted from
    the GUI / docs with minimal edits (use `odrv`, not `odrv0`):

      odrv.axis0.config.motor.pole_pairs = 20
      odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
      odrv.config.dc_max_positive_current = math.inf
    """
    g: dict[str, Any] = {
        "__builtins__": __builtins__,
        "odrv": odrv,  # connected Fibre device object
        "math": math,  # for math.inf soft limits in configs
    }
    try:
        import odrive.enums as enums  # type: ignore
    except ImportError:
        # apply_config will fail on first enum use; import_odrive() usually runs first.
        return g

    # Expose MotorType, ControlMode, InputMode, Protocol, EncoderId, AxisState, …
    for name in dir(enums):
        if name.startswith("_"):
            continue
        g[name] = getattr(enums, name)
    return g


def apply_config(config_path: Path, odrv: Any) -> None:
    """Run the config file against the connected device.

    Re-executes the whole file (including SERIAL_NUMBER = "..."), which is fine:
    that assignment only sets a local name and does not touch the device.
    Any Fibre write errors propagate as exceptions to main().
    """
    source = config_path.read_text(encoding="utf-8")
    g = build_config_globals(odrv)
    print(f"applying config from {config_path} ...")
    # Same globals+locals dict so assignments like `x = 1` are visible if needed.
    exec(compile(source, str(config_path), "exec"), g, g)  # noqa: S102
    print("config applied")


# ---------------------------------------------------------------------------
# Fibre-over-CAN (odrive Python package)
# ---------------------------------------------------------------------------
#
# Linux: interfaces=["can:can0"] (or can_core / can_payload on the rover).
# Same bus may already be used by custom_odrive_node — that is OK on SocketCAN;
# we park the node first so it is not sending setpoints during this session.
#


def import_odrive():
    """Import the odrive package or exit with install instructions.

    Not an apt/rosdep package — must be pip-installed in the environment
    (including inside the kanga Docker image if that is where you run this).
    """
    try:
        import odrive  # type: ignore
    except ImportError as exc:
        raise SystemExit(
            "odrive Python package not found. Install with:\n"
            "  python3 -m pip install --upgrade odrive\n"
            "(need >=0.6.11.post0 for Fibre-over-CAN; ODrive FW >=0.6.11)"
        ) from exc
    return odrive


def connect_odrive(odrive_mod: Any, can: str, serial: str, timeout_s: float) -> Any:
    """Find ODrive by serial on SocketCAN; always fail after timeout_s.

    Wrong SERIAL_NUMBER must not hang forever. find_any() takes a timeout kwarg,
    but in practice the calling thread can still block indefinitely on
    Future.result(), so we enforce our own wall clock and print progress.
    """
    from concurrent.futures import ThreadPoolExecutor
    from concurrent.futures import TimeoutError as FuturesTimeout

    # Fibre interface string — see odrive docs: "can:can0", not bare "can0".
    iface = f"can:{can}"
    # Slightly longer wall clock than find_any's own timeout so we outlast it.
    wall_s = timeout_s + 2.0
    print(f"connecting to ODrive serial={serial} on {iface} (timeout {timeout_s:.0f}s) ...")

    def _find() -> Any:
        # Runs on a worker thread so the main thread can poll + print status.
        return odrive_mod.find_any(
            serial_number=serial,
            interfaces=[iface],
            timeout=timeout_s,
        )

    # Do not use `with ThreadPoolExecutor`: on timeout its __exit__ calls
    # shutdown(wait=True), which waits for the still-running find_any worker and
    # can hang again (observed). Always shutdown(wait=False) instead.
    pool = ThreadPoolExecutor(max_workers=1)
    odrv: Any = None
    try:
        fut = pool.submit(_find)
        started = time.monotonic()
        next_ping = started + 5.0  # first "still searching" message after 5s
        while True:
            elapsed = time.monotonic() - started
            remaining = wall_s - elapsed
            if remaining <= 0:
                raise SystemExit(
                    f"timed out after {timeout_s:.0f}s waiting for ODrive serial={serial} on {iface}. "
                    "Check SERIAL_NUMBER in the config, CAN wiring, and that the drive is powered."
                )
            try:
                # Poll in ≤1s slices so we can print progress without busy-waiting.
                odrv = fut.result(timeout=min(1.0, remaining))
                break
            except FuturesTimeout:
                # Worker still searching — periodic keepalive message for the operator.
                now = time.monotonic()
                if now >= next_ping:
                    left = max(0.0, timeout_s - (now - started))
                    print(
                        f"  still searching for serial={serial} on {iface} "
                        f"({now - started:.0f}s elapsed, ~{left:.0f}s left) ..."
                    )
                    next_ping = now + 5.0
            except Exception as exc:  # noqa: BLE001
                raise SystemExit(f"failed to find ODrive {serial} on {iface}: {exc}") from exc
    finally:
        # Detach without waiting; process exit will reclaim the worker if needed.
        pool.shutdown(wait=False, cancel_futures=True)

    if odrv is None:
        raise SystemExit(f"no ODrive with serial {serial} found on {iface}")
    print(f"connected to ODrive {serial}")
    return odrv


def run_calibration(odrv: Any, timeout_s: float) -> None:
    """IDLE → clear errors → FULL_CALIBRATION_SEQUENCE; fail on non-success.

    Motor will energize and move — caller must have already confirmed the wheel
    is off the ground. Watchdog can still trip if cal is slow and watchdog is
    enabled in the just-applied config; clear_errors helps after a prior fault,
    and run_state feeds the watchdog while it polls.
    """
    try:
        from odrive.enums import AxisState, ProcedureResult  # type: ignore
    except ImportError as exc:
        raise SystemExit(f"cannot import AxisState from odrive.enums: {exc}") from exc

    try:
        from odrive.utils import request_state, run_state  # type: ignore
    except ImportError:
        # Older / incomplete installs: fall back to raw requested_state + poll.
        request_state = None
        run_state = None

    # Calibration and save both require IDLE.
    print("requesting IDLE before calibration ...")
    if request_state is not None:
        request_state(odrv.axis0, AxisState.IDLE)
    else:
        odrv.axis0.requested_state = AxisState.IDLE
    time.sleep(0.5)

    # Latched faults (esp. WATCHDOG_TIMER_EXPIRED: 0x01000000) block calibration.
    try:
        odrv.clear_errors()
        print("cleared errors before calibration")
    except Exception as exc:  # noqa: BLE001
        print(f"warning: clear_errors failed ({exc}); continuing")

    print("starting FULL_CALIBRATION_SEQUENCE ...")
    # Prefer odrive.utils.run_state: clears pending errors, enters the state, and
    # feeds the axis watchdog at ~5 Hz until the procedure finishes.
    if run_state is not None:
        try:
            run_state(odrv.axis0, AxisState.FULL_CALIBRATION_SEQUENCE)
        except Exception as exc:  # noqa: BLE001
            raise SystemExit(f"calibration failed: {exc}") from exc
        print("calibration finished")
        return

    # Fallback if utils is unavailable: set state and poll until IDLE + not BUSY.
    odrv.axis0.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    deadline = time.monotonic() + timeout_s
    busy = int(ProcedureResult.BUSY)  # 1
    success = int(ProcedureResult.SUCCESS)  # 0
    while time.monotonic() < deadline:
        try:
            state = int(odrv.axis0.current_state)
            result = int(odrv.axis0.procedure_result)
        except Exception as exc:  # noqa: BLE001
            raise SystemExit(f"lost ODrive while waiting for calibration: {exc}") from exc
        # Done when back in IDLE and procedure_result is no longer BUSY.
        if state == int(AxisState.IDLE) and result != busy:
            if result == success:
                print("calibration finished (procedure_result=SUCCESS)")
                return
            raise SystemExit(f"calibration ended with procedure_result={result}")
        time.sleep(0.2)
    raise SystemExit(f"calibration timed out after {timeout_s:.0f}s")


def save_configuration(odrv: Any) -> None:
    """Persist RAM config to NVRAM and reboot the ODrive.

    After reboot:
      - CAN heartbeats drop briefly; custom_odrive_node usually recovers on its own
      - Enable latch on the ROS node is left as we parked it (disabled) — we do
        not call set_enabled(true); the operator re-arms when ready
    Fibre often raises DeviceLost / disconnect when the board reboots; that is
    success, not failure.
    """
    print("calling save_configuration() (device will reboot) ...")
    try:
        odrv.save_configuration()
    except Exception as exc:  # noqa: BLE001
        name = type(exc).__name__
        if "DeviceLost" in name or "disconnected" in str(exc).lower():
            print(f"save_configuration: device disconnected as expected ({name})")
            return
        raise SystemExit(f"save_configuration failed: {exc}") from exc
    print("save_configuration returned")


# ---------------------------------------------------------------------------
# Coexistence with custom_odrive_node (ROS)
# ---------------------------------------------------------------------------
#
# custom_odrive_node only sends CAN when it has work (control_message, services).
# If it is still enabled with a publisher streaming setpoints, those frames race
# with Fibre config / calibration. ParkHelper disables that motor first.
#


class ParkHelper:
    """Park a sibling custom_odrive_node and check /drivestop.

    SocketCAN allows both this script and the C++ node on the same bus; the
    risk is command races (setpoints / closed-loop fighting Fibre). Prefer
    --ns so we disable + IDLE the matching motor before commissioning.

    After park we shut this helper down before Fibre work — we intentionally
    never re-enable the motor from this script.
    """

    def __init__(self, namespace: str | None) -> None:
        # Late imports so `commission --help` still works if ROS is not sourced
        # (main always uses ros2 run, which has the env).
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool
        from std_srvs.srv import SetBool

        from custom_odrive.msg import ControllerStatus
        from custom_odrive.srv import AxisState

        self._rclpy = rclpy
        self._ns = normalize_ns(namespace) if namespace else None
        # Defaults until first controller_status (or remain unused in bench mode).
        self._enabled = True
        self._axis_state = -1
        self._drivestop: bool | None = None  # None = no latch message received yet

        rclpy.init()
        self._node = Node("custom_odrive_commission")

        # Must match custom_odrive_node subscriber QoS or we never see the latch.
        latch_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._node.create_subscription(Bool, "/drivestop", self._on_drivestop, latch_qos)

        self._cli_enable = None
        self._cli_state = None
        if self._ns:
            # Node publishes controller_status as best effort — must match.
            status_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self._node.create_subscription(
                ControllerStatus,
                f"{self._ns}/controller_status",
                self._on_status,
                status_qos,
            )
            self._cli_enable = self._node.create_client(SetBool, f"{self._ns}/set_enabled")
            self._cli_state = self._node.create_client(AxisState, f"{self._ns}/request_axis_state")

    def _on_drivestop(self, msg) -> None:
        self._drivestop = bool(msg.data)

    def _on_status(self, msg) -> None:
        self._enabled = bool(msg.enabled)
        self._axis_state = int(msg.axis_state)

    def spin_brief(self, seconds: float = 0.5) -> None:
        """Pump DDS briefly so latch / status callbacks can arrive."""
        deadline = time.monotonic() + seconds
        while self._rclpy.ok() and time.monotonic() < deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.05)

    def check_drivestop(self) -> None:
        """Abort if the global stop latch is asserted.

        Only aborts on explicit True. If no publisher has ever latched a value,
        _drivestop stays None and we proceed (same default-off behavior as the node).
        """
        self.spin_brief(0.5)
        if self._drivestop is True:
            raise SystemExit(
                "/drivestop is asserted (true) — clear it before commissioning"
            )

    def park_or_confirm(self) -> None:
        """Disable+IDLE via --ns, or interactively confirm bench mode if --ns omitted."""
        self.check_drivestop()

        # ----- Bench mode (no --ns) -----
        # Operator must confirm nothing is commanding this motor. Saying "y" while
        # a live custom_odrive_node + control publisher is still running is unsafe.
        if self._ns is None:
            if not prompt_yes(
                "custom_odrive_node is not commanding this motor "
                "(no control publisher / node stopped)? [y/N]: "
            ):
                raise SystemExit("aborted: coexistence not confirmed")
            print("bench mode: proceeding without parking a ROS node")
            return

        # ----- ROS park mode (--ns given) -----
        assert self._cli_enable is not None and self._cli_state is not None
        print(f"waiting for services under {self._ns} ...")
        deadline = time.monotonic() + 5.0
        while self._rclpy.ok() and time.monotonic() < deadline:
            if self._cli_enable.service_is_ready() and self._cli_state.service_is_ready():
                break
            self._rclpy.spin_once(self._node, timeout_sec=0.1)
        else:
            # --ns was requested but nothing is listening — usually forgot to launch,
            # or wrong namespace. Force an explicit choice (start node or omit --ns).
            raise SystemExit(
                f"services not available under {self._ns} — "
                "start that motor's custom_odrive_node, or omit --ns for bench-only"
            )

        from std_srvs.srv import SetBool

        from custom_odrive.srv import AxisState

        AXIS_STATE_IDLE = 1

        # 1) Disable latch: node ignores control_message and requests IDLE itself.
        req = SetBool.Request()
        req.data = False
        fut = self._cli_enable.call_async(req)
        self._rclpy.spin_until_future_complete(self._node, fut, timeout_sec=5.0)
        if not fut.done() or fut.result() is None or not fut.result().success:
            raise SystemExit(f"set_enabled(false) failed under {self._ns}")
        print(f"parked: set_enabled(false) on {self._ns}")

        # 2) Explicit IDLE in case the axis was in another non-busy state.
        state_req = AxisState.Request()
        state_req.axis_requested_state = AXIS_STATE_IDLE
        fut2 = self._cli_state.call_async(state_req)
        self._rclpy.spin_until_future_complete(self._node, fut2, timeout_sec=15.0)
        if not fut2.done() or fut2.result() is None:
            raise SystemExit(f"request_axis_state(IDLE) failed under {self._ns}")

        # 3) Best-effort confirm via status. Cyclic messages may be sparse right
        # after disable; service success is enough to continue if we never see it.
        wait_deadline = time.monotonic() + 5.0
        while self._rclpy.ok() and time.monotonic() < wait_deadline:
            self._rclpy.spin_once(self._node, timeout_sec=0.05)
            if (not self._enabled) and self._axis_state == AXIS_STATE_IDLE:
                print(f"confirmed disabled + IDLE on {self._ns}")
                return
        print(
            f"warning: did not observe enabled=false/IDLE on controller_status "
            f"(enabled={self._enabled}, axis_state={self._axis_state}); "
            "continuing after successful park services"
        )

    def shutdown(self) -> None:
        """Tear down the short-lived rclpy node (does not re-enable the motor)."""
        try:
            self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if self._rclpy.ok():
            self._rclpy.shutdown()


# ---------------------------------------------------------------------------
# CLI + main
# ---------------------------------------------------------------------------


def parse_args(argv: list[str]) -> argparse.Namespace:
    """CLI after `ros2 run custom_odrive commission -- ...`."""
    p = argparse.ArgumentParser(
        description="Apply ODrive motor config over SocketCAN (Fibre-over-CAN)"
    )
    p.add_argument(
        "--can",
        required=True,
        help="SocketCAN interface (e.g. can0, can_core, can_payload)",
    )
    p.add_argument(
        "--config",
        required=True,
        type=Path,
        help="Path to motor config .py (must define SERIAL_NUMBER)",
    )
    p.add_argument(
        "--ns",
        default=None,
        help="ROS namespace of sibling custom_odrive_node to park (e.g. /wheel_fl)",
    )
    p.add_argument(
        "--calibrate",
        action="store_true",
        help="Run FULL_CALIBRATION_SEQUENCE after applying config",
    )
    p.add_argument(
        "--save",
        action="store_true",
        help="Call save_configuration() (device reboots)",
    )
    p.add_argument(
        "--timeout",
        type=float,
        default=180.0,
        help="Timeout seconds for calibration (default: 180). "
        "Connect discovery is separately capped at 30s.",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """Entry point. Returns process exit code (0 success, 1 failure)."""
    args = parse_args(argv if argv is not None else sys.argv[1:])
    config_path = args.config.expanduser().resolve()
    if not config_path.is_file():
        print(f"config not found: {config_path}", file=sys.stderr)
        return 1

    # --- 1) Identity: SERIAL_NUMBER from config only (no --serial CLI) ---
    serial = read_serial_number(config_path)
    print(f"target SERIAL_NUMBER={serial} from {config_path}")

    # --- 2) Park ROS sibling / confirm bench (before Fibre, so no setpoint races) ---
    park: ParkHelper | None = None
    try:
        park = ParkHelper(args.ns)
        park.park_or_confirm()
    except SystemExit:
        # Helper raised SystemExit("message") — still need to tear down rclpy.
        if park is not None:
            park.shutdown()
        raise
    except Exception as exc:  # noqa: BLE001
        if park is not None:
            park.shutdown()
        print(f"ROS park/coexistence check failed: {exc}", file=sys.stderr)
        return 1

    # ROS clients no longer needed during Fibre work; shut down to free DDS
    # before the odrive device manager starts its own background threads.
    if park is not None:
        park.shutdown()
        park = None

    # --- 3) Fibre connect (serial-targeted) ---
    odrive_mod = import_odrive()
    # Cap discovery wait so a wrong SERIAL_NUMBER fails in ~30s, not full --timeout
    # (which is meant for long calibrations).
    odrv = connect_odrive(odrive_mod, args.can, serial, min(args.timeout, 30.0))

    try:
        # --- 4) Apply config assignments into RAM on the device ---
        apply_config(config_path, odrv)

        # --- 5) Optional calibration (safety prompt first) ---
        if args.calibrate:
            if not prompt_yes(
                "Calibration will energize and move the motor.\n"
                "Confirm the wheel is OFF the ground and free to spin. Continue? [y/N]: "
            ):
                # Config may already be applied in RAM but not saved — intentional:
                # operator can re-run with --save later, or power-cycle to discard.
                print("aborted: calibration not confirmed", file=sys.stderr)
                return 1
            run_calibration(odrv, args.timeout)

        # --- 6) Optional NVRAM save (reboots the drive) ---
        if args.save:
            save_configuration(odrv)

        print("commission complete (motor left disabled if a ROS node was parked)")
        return 0
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001
        print(f"commission failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit as exc:
        # Helpers use raise SystemExit("human message") for expected failures.
        # Map string codes to stderr + exit 1; preserve integer codes as-is.
        code = exc.code
        if code is None:
            sys.exit(0)
        if isinstance(code, int):
            sys.exit(code)
        print(code, file=sys.stderr)
        sys.exit(1)
