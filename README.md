# custom-ros-odrive

ROS 2 packages for controlling ODrive motor controllers over CAN (SocketCAN).

Based on [odriverobotics/ros_odrive](https://github.com/odriverobotics/ros_odrive). Not an official ODrive product. MIT license — see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE).

## Packages

- **`custom_odrive`** — per-motor ROS 2 node + commissioning / test scripts
- **`odrive_base`** — SocketCAN + epoll helpers (compiled into the node)

Interface details: [`custom_odrive/README.md`](custom_odrive/README.md).  
Parameter defaults: [`custom_odrive/config/custom_odrive_defaults.yaml`](custom_odrive/config/custom_odrive_defaults.yaml).

## Build

```bash
cd /path/to/custom-ros-odrive
colcon build --packages-select custom_odrive
source install/setup.bash
```

Requires ROS 2 Humble or newer. Dev Containers are under `.devcontainer/`.

## Launch

Host must bring SocketCAN up first (e.g. `can0`).

```bash
ros2 launch custom_odrive example_launch.py         # one motor (wheel_bl, node_id 3)
ros2 launch custom_odrive example_multi_launch.py   # four motors (wheel_fl/bl/br/fr)
```

Topics/services are under each Node’s namespace (e.g. `/wheel_fl/...`).

## Runtime use

Examples below use `/wheel_fl` from `example_multi_launch.py` with `control_message_in_radians: true`.

| Name | Value |
|------|-------|
| IDLE | `1` |
| CLOSED_LOOP_CONTROL | `8` |
| ControlMode velocity | `2` |
| InputMode vel ramp | `2` |

```bash
# Feedback
ros2 topic echo /wheel_fl/controller_status

# Enable / disable
ros2 service call /wheel_fl/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /wheel_fl/set_enabled std_srvs/srv/SetBool "{data: false}"

# Closed-loop / idle
ros2 service call /wheel_fl/request_axis_state custom_odrive/srv/AxisState "{axis_requested_state: 8}"
ros2 service call /wheel_fl/request_axis_state custom_odrive/srv/AxisState "{axis_requested_state: 1}"

# Velocity (rad/s) — publish continuously; each message feeds the ODrive watchdog
ros2 topic pub -r 10 /wheel_fl/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 2, input_mode: 2, input_pos: 0.0, input_vel: 6.283185, input_torque: 0.0}"
```

Global stop (all motors):

```bash
ros2 topic pub --once /drivestop std_msgs/msg/Bool "{data: true}"   # IDLE + block motion
ros2 topic pub --once /drivestop std_msgs/msg/Bool "{data: false}"  # allow again (does not auto-enable)
```

Helper test (streams setpoints and confirms `axis_state == 8`):

```bash
ros2 run custom_odrive velocity_ramp_test -- --ns /wheel_fl --target-vel 6.28
```

### Watchdog

The node does not send a periodic keepalive. Publish `control_message` at roughly **5–10×** the firmware `watchdog_timeout` (e.g. ≥5–10 Hz for a 1 s watchdog), or the drive returns to IDLE when setpoints stop.

## Commissioning

Apply firmware config over SocketCAN (Fibre / odrivetool-over-CAN), optionally calibrate and save:

```bash
python3 -m pip install --upgrade odrive   # >= 0.6.11.post0; drive FW >= 0.6.11

ros2 run custom_odrive commission -- \
  --can can0 \
  --config /path/to/wheel_fl_motor_config.py \
  --ns /wheel_fl \
  --calibrate \
  --save
```

Config files live under [`custom_odrive/config/`](custom_odrive/config/) (`SERIAL_NUMBER` required in each file). See [`custom_odrive/README.md`](custom_odrive/README.md#commissioning).

## Compatible devices

ODrive Pro, S1, Micro (not ODrive 3.x). Firmware cyclic-message setup: [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).
