# custom_odrive

Per-motor ROS 2 node for an ODrive over CAN Simple (SocketCAN). Also provides commissioning and velocity test scripts.

Build / walkthrough: [repository README](../README.md).

## Interface

### Subscribes

* `control_message` (`custom_odrive/msg/ControlMessage`) — setpoints  
  Ignored while disabled (`set_enabled`) or while `/drivestop` is `true`.
* `/drivestop` (`std_msgs/msg/Bool`, absolute, reliable + transient local) — global stop  
  - Default: off until a message is received  
  - `true` → IDLE + block motion / enable / non-IDLE state requests  
  - `false` → allow again (does not auto-enable or enter closed loop)

### Publishes

* `controller_status` (`custom_odrive/msg/ControllerStatus`) — estimates, axis state, `enabled` latch  
* `odrive_status` (`custom_odrive/msg/ODriveStatus`) — errors / temps / bus (needs those cyclic messages enabled in firmware)

### Services

* `request_axis_state` (`custom_odrive/srv/AxisState`) — axis state transition; response has `success` / `timed_out`  
  Non-IDLE requests rejected while disabled or `/drivestop` is true  
* `clear_errors` (`std_srvs/srv/Empty`)
* `set_enabled` (`std_srvs/srv/SetBool`) — enable latch; `false` requests IDLE and ignores `control_message`
* `get_errors` (`custom_odrive/srv/GetErrors`) — last error bitfields + decoded names

## Parameters / launch

Defaults: [`config/custom_odrive_defaults.yaml`](config/custom_odrive_defaults.yaml).

Required per Node (not in the defaults file):

| Field | Where |
|-------|--------|
| `namespace` | `Node(..., namespace=...)` |
| `node_id` | parameters |
| `interface` | parameters |

```bash
ros2 launch custom_odrive example_launch.py         # wheel_bl, node_id 3
ros2 launch custom_odrive example_multi_launch.py   # wheel_fl/bl/br/fr, node_ids 1–4
```

Copy a launch into the rover package for production; keep `node_id` aligned with each motor’s commissioned CAN id.

## Scripts

### `velocity_ramp_test`

Enable, enter closed loop, stream velocity setpoints (feeds watchdog):

```bash
ros2 run custom_odrive velocity_ramp_test -- --ns /wheel_fl --target-vel 6.28
```

### `commission`

Apply a motor config over SocketCAN via Fibre ([odrivetool-over-CAN](https://docs.odriverobotics.com/v/latest/interfaces/odrivetool.html#using-odrivetool-via-can)).

Requirements: `pip install --upgrade odrive` (≥0.6.11.post0), ODrive FW ≥0.6.11, SocketCAN up.

```bash
ros2 run custom_odrive commission -- \
  --can can0 \
  --config $(ros2 pkg prefix custom_odrive)/share/custom_odrive/config/wheel_fl_motor_config.py \
  --ns /wheel_fl \
  --calibrate \
  --save
```

| Flag | Meaning |
|------|---------|
| `--can` | SocketCAN interface (`can0`, `can_core`, …) |
| `--config` | Motor `.py` with required `SERIAL_NUMBER` + `odrv.*` assignments |
| `--ns` | Park that motor’s node (`set_enabled false` + IDLE) before Fibre work |
| `--calibrate` | Full calibration (prompts that the wheel is off the ground) |
| `--save` | `save_configuration()` (drive reboots) |

Without `--ns`, the script asks you to confirm nothing is commanding the motor. After commission the motor is left disabled; re-enable with `set_enabled` when ready.

Example / wheel configs: [`config/`](config/).
