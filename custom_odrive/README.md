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
