# custom_odrive Testing Plan

Manual / hardware-in-the-loop (HIL) test checklist for the `custom_odrive` node.
There are no automated tests in this repo (CI only builds), so this plan is
executed by hand against real hardware.

## Test rig assumptions

- Bus: `can0` @ **500000** bit/s
- Motor under test: `node_id: 2`, ROS namespace `wheel_bl` (topics/services under `/wheel_bl/...`)
- Host: Nvidia (Jetson-class), ROS 2 (Humble or newer)
- All CAN IDs for this motor are `(2 << 5) | cmd = 0x40 | cmd`
  (heartbeat `0x041`, encoder `0x049`, set-state `0x047`, etc.)

Example launch parameters:

```python
namespace="wheel_bl",
parameters=[
    defaults,
    {"node_id": 2, "interface": "can0"},
],
```

## Firmware prerequisite

The ODrive must be configured to **cyclically transmit** these messages or the
feedback topics never publish:
heartbeat, encoder estimates, Iq, torques, error, temperature, bus voltage/current.
See the ODrive ROS CAN package guide.

## Watchdog / publish rate

This node has NO periodic/keepalive CAN traffic. The ODrive axis watchdog is fed only when
the node transmits a setpoint, which happens once per `control_message` you publish. So
staying armed in CLOSED_LOOP depends entirely on your publish rate:

- Publish `control_message` with margin - about 5-10x the watchdog rate. For a 1 s watchdog,
  publish at >= 5-10 Hz so a few dropped/late messages don't trip it.
- If the publisher is remote (over wifi), add more margin or keep the control loop on-board.
- Publisher QoS must be compatible with the subscriber (`KeepLast(1)`, reliable by default),
  or messages won't arrive at all and the axis will disarm.
- Stopping publishing, or `set_enabled(false)`, stops setpoints -> watchdog trips ~1 timeout
  later and the motor disarms to idle. This is the intended safety behavior.

Confirm the ODrive firmware has the CAN watchdog enabled (`enable_watchdog` + `watchdog_timeout`)
so comms loss faults the motor to idle.

## Safety rules

- Do Phases 0-1 with the motor unpowered or the wheel off the ground.
- Power the motor only from Phase 2 on, with an **e-stop / power cutoff in reach**
  and the wheel free to spin (bot on blocks).
- Keep setpoints tiny in Phase 3 and ramp up slowly.
- Run `ros2 bag record -a` during HIL sessions for post-hoc review.

---

## Every session - setup

Run once per new terminal before any test:

```bash
# CAN bus up at 500k (skip if already configured by systemd/netplan)
sudo ip link set can0 up type can bitrate 500000
ip -details link show can0

# Build + source the workspace
colcon build --packages-select custom_odrive
source install/setup.bash

# Launch the node under test (node_id 2, ns wheel_bl)
ros2 launch custom_odrive example_launch.py
```

Handy shell alias so the control-mode commands below stay short:

```bash
NS=/wheel_bl
```

---

## Phase 0 - Pre-flight (no motor power)

- [ ] 0.1 Package builds (clean, no errors)

```bash
colcon build --packages-select custom_odrive
```

- [ ] 0.2 Interfaces generated (3 msgs + `AxisState`)

```bash
ros2 interface list | grep custom_odrive
```

- [ ] 0.3 CAN link up @ 500k (state UP, bitrate 500000)

```bash
ip -details link show can0
```

- [ ] 0.4 Bus sees ODrive: cyclic heartbeat at `0x041`

```bash
candump can0
# or only this motor's heartbeat:
candump can0,041:7ff
```

- [ ] 0.5 Firmware cyclic TX confirmed: encoder, Iq, torques, error, temp, bus V/I all visible

```bash
# watch all frames from node_id 2 (IDs 0x040-0x05f)
candump can0,040:7e0
```

## Phase 1 - Node bring-up & telemetry (motor unpowered / free-spinning)

- [ ] 1.1 Node launches: log prints `node_id: 2`, `interface: can0`, `start_enabled: true`, no CAN init error

```bash
ros2 launch custom_odrive example_launch.py
```

- [ ] 1.2 Topics/services present under `/wheel_bl/...`

```bash
ros2 topic list | grep wheel_bl
ros2 service list | grep wheel_bl
```

- [ ] 1.3 `controller_status` publishing (live pos/vel, sensible `axis_state`, `enabled: true`)

```bash
ros2 topic echo /wheel_bl/controller_status
```

- [ ] 1.4 `odrive_status` publishing (real `bus_voltage`, `fet_temperature`; needs error+temp+busV cyclic)

```bash
ros2 topic echo /wheel_bl/odrive_status
```

- [ ] 1.5 node_id filtering: with another ODrive on the bus, this node only reflects id 2 (compare echo above against `candump can0`)
- [ ] 1.6 Radian conversion: hand-rotate one full turn -> `pos_estimate` ~= 6.283 (2pi), not 1.0

```bash
ros2 topic echo /wheel_bl/controller_status --field pos_estimate
```

## Phase 2 - State machine & services (motor powered, wheel free, e-stop ready)

- [ ] 2.1 Request CLOSED_LOOP -> `success: true`, `axis_state: 8` (note >=1s min wait)

```bash
ros2 service call /wheel_bl/request_axis_state custom_odrive/srv/AxisState "{axis_requested_state: 8}"
```

- [ ] 2.2 Request IDLE -> `success: true`, `axis_state: 1`

```bash
ros2 service call /wheel_bl/request_axis_state custom_odrive/srv/AxisState "{axis_requested_state: 1}"
```

- [ ] 2.3 Timeout path (OPTIONAL - skip in normal use)

  The call completes early when `(requested_closed_loop || !is_busy) && >=1s`, so a
  CLOSED_LOOP (8) or IDLE (1) request always "succeeds" after ~1s and can NOT time out.
  `timed_out: true` only happens when a state stays `procedure_result == BUSY` longer than
  the timeout, i.e. a calibration procedure (3/4/7). We don't run calibration through this
  service, so this path is not exercised in normal operation - skip unless specifically
  validating calibration handling (calibration energizes/spins the motor).

- [ ] 2.4a Read errors first: trip a fault, then confirm `active_errors` is non-zero

```bash
# odrive_status carries the GetError active_errors + disarm_reason
ros2 topic echo --once /wheel_bl/odrive_status
# (controller_status.active_errors from heartbeat should also be non-zero)
ros2 topic echo --once /wheel_bl/controller_status
```

- [ ] 2.4b `clear_errors`: clear, then confirm `active_errors` returns to 0

```bash
ros2 service call /wheel_bl/clear_errors std_srvs/srv/Empty "{}"
ros2 topic echo --once /wheel_bl/odrive_status   # active_errors == 0
```

- [ ] 2.5 Disable latch -> motor IDLE, `enabled: false`

```bash
ros2 service call /wheel_bl/set_enabled std_srvs/srv/SetBool "{data: false}"
```

- [ ] 2.6 Control blocked while disabled -> no motion

```bash
ros2 topic pub --once /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 2, input_mode: 2, input_pos: 0.0, input_vel: 1.0, input_torque: 0.0}"
```

- [ ] 2.7 Non-IDLE state blocked while disabled -> `success: false`

```bash
ros2 service call /wheel_bl/request_axis_state custom_odrive/srv/AxisState "{axis_requested_state: 8}"
```

- [ ] 2.8 Re-enable, then request state 8 -> accepted

```bash
ros2 service call /wheel_bl/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /wheel_bl/request_axis_state custom_odrive/srv/AxisState "{axis_requested_state: 8}"
```

## Phase 3 - Control modes (bench-secured motor, low limits, e-stop in hand)

Enable + set CLOSED_LOOP before each. Start with tiny setpoints.

```bash
ros2 service call /wheel_bl/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /wheel_bl/request_axis_state custom_odrive/srv/AxisState "{axis_requested_state: 8}"
```

- [ ] 3.1 Velocity mode -> spins at commanded rad/s (check `vel_estimate` vs command)

```bash
ros2 topic pub /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 2, input_mode: 2, input_pos: 0.0, input_vel: 1.0, input_torque: 0.0}"
```

- [ ] 3.2 Torque mode -> expected torque

```bash
ros2 topic pub /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 1, input_mode: 1, input_pos: 0.0, input_vel: 0.0, input_torque: 0.05}"
```

- [ ] 3.3 Position mode, zero FF -> moves to commanded position

```bash
ros2 topic pub --once /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 3, input_mode: 3, input_pos: 1.57, input_vel: 0.0, input_torque: 0.0}"
```

- [ ] 3.4 Position mode, non-zero FF -> validates the int16 feed-forward fix

  Confirms the Set_Input_Pos frame (cmd 0x00C -> CAN ID 0x04C for node 2) packs the
  feed-forward as int16, not the old truncated int8. Watch the raw 8 bytes:
    bytes 0-3 = position (float32)
    bytes 4-5 = vel FF   (int16 LE, = rev/s * 1000)
    bytes 6-7 = torque FF (int16 LE, = Nm * 1000)
  With control_message_in_radians, input_vel is converted rad/s -> rev/s (/2pi) before *1000.
  Example below: input_vel 2.0 rad/s -> 2.0/2pi*1000 ~= 318 = 0x013E -> bytes "3E 01";
  input_torque 0.1 Nm -> 100 = 0x0064 -> bytes "64 00". Exact values are fiddly; the pass
  criterion is simply that bytes 4-7 are NON-ZERO and change with the FF (pre-fix the high
  byte stayed 0 / values were truncated). Compare against a zero-FF command to see the diff.

```bash
# Terminal A - watch the wire (leave running, Ctrl-C when done):
candump can0,04c:7ff

# Terminal B - send non-zero FF, then zero FF, and compare bytes 4-7:
ros2 topic pub --once /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 3, input_mode: 3, input_pos: 1.57, input_vel: 2.0, input_torque: 0.1}"
# expect a 0x04C frame ending roughly in: ... 3E 01 64 00  (bytes 4-7 non-zero)
ros2 topic pub --once /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 3, input_mode: 3, input_pos: 1.57, input_vel: 0.0, input_torque: 0.0}"
# expect bytes 4-7 = 00 00 00 00
```

- [ ] 3.5 Controller-mode frame sent only on change (separate concern from 3.4)

  The node sends Set_Controller_Mode (cmd 0x00B -> CAN ID 0x04B) only when control_mode or
  input_mode changes, not on every control_message. Watch 0x04B: publishing repeatedly in
  the SAME mode should show it ONCE; switching mode should make it appear AGAIN. The node
  must be enabled (set_enabled true) or it sends nothing. Use TWO terminals - do NOT chain
  candump with `&` (running it repeatedly just spawns duplicate listeners; `pkill candump`
  to clean up if you did).

```bash
# Terminal A - watch the wire (leave running):
candump can0,04b:7ff

# Terminal B - publish repeatedly in ONE mode -> expect a single 0x04B frame:
ros2 topic pub -r 5 /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 2, input_mode: 2, input_pos: 0.0, input_vel: 1.0, input_torque: 0.0}"
# then switch mode -> expect ONE new 0x04B frame:
ros2 topic pub --once /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 1, input_mode: 1, input_pos: 0.0, input_vel: 0.0, input_torque: 0.05}"
```

- [ ] 3.6 `invert_direction: true` relaunch: command and feedback signs both flip consistently

```bash
ros2 run custom_odrive custom_odrive_node --ros-args -r __ns:=/wheel_bl -r __node:=can_node \
  -p node_id:=2 -p interface:=can0 -p invert_direction:=true
```

## Phase 4 - Safety, faults & lifecycle

- [ ] 4.1 Shutdown -> IDLE: Ctrl-C node while in CLOSED_LOOP (`axis_idle_on_shutdown: true`), confirm IDLE (`0x047`) on the wire

```bash
candump can0,047:7ff   # in a second terminal, then Ctrl-C the node
```

- [ ] 4.2 Shutdown w/o IDLE: relaunch with the flag off, Ctrl-C -> no IDLE sent

```bash
ros2 run custom_odrive custom_odrive_node --ros-args -r __ns:=/wheel_bl -r __node:=can_node \
  -p node_id:=2 -p interface:=can0 -p axis_idle_on_shutdown:=false
```

- [ ] 4.3 CAN drop: node handles gracefully, no crash-loop

```bash
sudo ip link set can0 down
# ... observe node logs ...
sudo ip link set can0 up type can bitrate 500000
```

- [ ] 4.4 Cable/power yank mid-motion -> motor faults/stops, node survives (physical test; watch node logs)
- [ ] 4.5 Malformed frame -> `verify_length` warns, no crash

```bash
# wrong-length heartbeat (should be 8 bytes) for node_id 2
cansend can0 041#0011
```

- [ ] 4.6 Watchdog trip: in CLOSED_LOOP, stop publishing control_message -> motor disarms to idle within ~1 watchdog timeout (requires firmware watchdog enabled)

```bash
# publish for a moment, then Ctrl-C the publisher and watch controller_status.axis_state -> 1
ros2 topic pub -r 10 /wheel_bl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 2, input_mode: 2, input_pos: 0.0, input_vel: 1.0, input_torque: 0.0}"
```

## Phase 5 - Multi-node / integration (only if running >1 wheel)

- [ ] 5.1 Two nodes on one bus: namespace isolation, each node reacts only to its id

```bash
ros2 launch custom_odrive example_multi_launch.py
```

- [ ] 5.2 Rate/latency: stable publish rate, acceptable command->motion latency under load

```bash
ros2 topic hz /wheel_bl/controller_status
```

---

## Known issue fixed during review

**Position-mode feed-forward truncation.** `Set_Input_Pos` (cmd `0x00C`) encodes
`Vel_FF` and `Torque_FF` as **int16** (bytes 4-5 and 6-7, scale 0.001). The node
previously wrote them as `int8_t` into bytes 4 and 6 only, which overflowed for any
FF beyond ~0.127 rev/s / Nm and left the high byte zeroed. Fixed to `int16_t` in
`custom_odrive/src/custom_odrive_node.cpp`. Phase 3.4 validates the fix.
