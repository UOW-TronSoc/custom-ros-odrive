# custom_odrive

Standalone per-motor ROS 2 node for controlling an ODrive over CAN.

Derived from the upstream [`odrive_node`](https://github.com/odriverobotics/ros_odrive/tree/main/odrive_node) package (`odrive_can`), with in-tree epoll + SocketCAN from `odrive_base`.

For build steps and a CLI walkthrough (closed loop, velocity, enable/disable), see the [repository README](../README.md).

## Interface

### Subscribes

* `control_message` (`custom_odrive/msg/ControlMessage`): setpoints for the ODrive  
  Ignored while the motor is disabled via `set_enabled` or while `/drivestop` is `true`.
* `/drivestop` (`std_msgs/msg/Bool`, absolute topic, reliable + transient local): global drive stop  
  - **Local default: OFF** — until a message is received, drivestop is not asserted and motion commands are allowed.  
  - `data: true` → request IDLE and block control / `set_enabled(true)` / non-IDLE state requests  
  - `data: false` → drivestop off; normal commands allowed again (does **not** auto-enable or enter closed loop)  
  - A long-lived system latch publisher (same QoS) should own the last value across restarts; this node only subscribes.

### Publishes

* `controller_status` (`custom_odrive/msg/ControllerStatus`): controller-level feedback, including `enabled` (enable latch)  
  Publishes when a full cyclic set arrives, or when encoder estimates alone (or heartbeat+encoder) are available.  
  Connectivity is inferred from whether this topic keeps updating (no separate health topic).
* `odrive_status` (`custom_odrive/msg/ODriveStatus`): system-level feedback (requires error/temp/bus cyclic messages)

### Services

* `request_axis_state` (`custom_odrive/srv/AxisState`): request an axis state transition  
  - Waits at least 1 s (same as upstream), then up to `request_axis_state_timeout_s`  
  - Response includes `success` and `timed_out`  
  - Non-IDLE requests are rejected while disabled (`success=false`, `timed_out=false`)
* `clear_errors` (`std_srvs/srv/Empty`): clear errors without changing axis state
* `set_enabled` (`std_srvs/srv/SetBool`): enable/disable latch for this motor  
  - `data: false` → disable, request IDLE, ignore `control_message`, reject non-IDLE state requests  
  - `data: true` → enable again (does **not** automatically enter closed loop)

## Launch

Shared defaults live in [`config/custom_odrive_defaults.yaml`](config/custom_odrive_defaults.yaml).  
Each launch `Node` loads that file, then a small dict of **required** params and any **overrides**.

**Required in every Node** (not in the defaults file):

| Field | Where |
|-------|--------|
| `namespace` | `Node(..., namespace=...)` |
| `node_id` | parameters dict |
| `interface` | parameters dict |

### Single motor (`example_launch.py`)

```bash
ros2 launch custom_odrive example_launch.py
```

### Multi motor (`example_multi_launch.py`)

Same structure with one `Node` block per motor (axis1 overrides `invert_direction`).

```bash
ros2 launch custom_odrive example_multi_launch.py
```

For a rover, copy the multi launch into the rover package, reuse the defaults YAML (or your own), and add one `Node` block per wheel.

For firmware cyclic-message setup, see the ODrive [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).

## Commissioning (config / calibrate / save over CAN)

One-shot script that uses the official [odrivetool-over-CAN](https://docs.odriverobotics.com/v/latest/interfaces/odrivetool.html#using-odrivetool-via-can) path (`odrive` Python Fibre-over-CAN on SocketCAN). No USB required.

**Requirements**
- `python3 -m pip install --upgrade odrive` (≥0.6.11.post0)
- ODrive firmware ≥0.6.11
- SocketCAN interface already up on the host (e.g. `can0`, or kanga `can_core` / `can_payload`)

**Motor config** — one `.py` per ESC. `SERIAL_NUMBER` is required and is the **only** identity source (no CLI serial override). Example: [`config/example_motor_config.py`](config/example_motor_config.py).

**Coexistence with `custom_odrive_node`**
- Prefer `--ns` matching that motor’s namespace so the script calls `set_enabled(false)` + IDLE before Fibre work
- Aborts if `/drivestop` is asserted
- Without `--ns`, confirms interactively that nothing is commanding the motor
- Does **not** re-enable after commission; use `set_enabled` when ready
- After `--save` the drive reboots (brief heartbeat dropout)

**Calibration** — with `--calibrate`, prompts to confirm the wheel is off the ground before starting `FULL_CALIBRATION_SEQUENCE`.

```bash
ros2 run custom_odrive commission -- \
  --can can_core \
  --config /path/to/motor_config.py \
  --ns /odrive_axis0 \
  --calibrate \
  --save
```

On a laptop with a single adapter, use `--can can0`. Same script works inside the kanga_wip Orin Docker image (`network_mode: host`, `privileged`) once SocketCAN is up on the host and `odrive` is pip-installed in the container.
