# custom-ros-odrive

Custom / modified ROS 2 packages based on the official ODrive Robotics [`ros_odrive`](https://github.com/odriverobotics/ros_odrive) project.

> **Not an official ODrive product.** This repository is independently maintained by UOW TronSoc and is not affiliated with, endorsed by, or sponsored by [ODrive Robotics](https://odriverobotics.com).

## Attribution and license

- **Upstream:** [odriverobotics/ros_odrive](https://github.com/odriverobotics/ros_odrive)
- **Upstream license:** MIT — Copyright (c) 2023 ODrive Robotics
- **This project:** MIT — see [`LICENSE`](LICENSE) and [`NOTICE`](NOTICE)

## Packages

- **`custom_odrive`**: Standalone per-motor ROS 2 node for ODrive communication via CAN (epoll + SocketCAN, same transport pattern as upstream).
- **`odrive_base`**: Shared epoll event loop and SocketCAN helpers (compiled into the node; not a separate ament package).

This repository does **not** include upstream `odrive_ros2_control` or the BotWheel explorer example.

See [`custom_odrive/README.md`](custom_odrive/README.md) for topics, services, launch layout, and safety behavior. Parameter defaults live in [`custom_odrive/config/custom_odrive_defaults.yaml`](custom_odrive/config/custom_odrive_defaults.yaml).

## Launch

```bash
ros2 launch custom_odrive example_launch.py        # one motor (odrive_axis0)
ros2 launch custom_odrive example_multi_launch.py  # two motors
```

## Build

Ubuntu with ROS 2 Humble or newer, or use the Dev Containers in `.devcontainer/`:

```bash
cd /path/to/custom-ros-odrive
colcon build --packages-select custom_odrive
source install/setup.bash
```

## Use

Examples assume `example_launch.py` (`/odrive_axis0/...`) with `control_message_in_radians: true` (the package default).

Useful enum values:

| Name | Value |
|------|-------|
| IDLE | `1` |
| CLOSED_LOOP_CONTROL | `8` |
| ControlMode velocity | `2` |
| InputMode vel ramp | `2` |

### Check feedback

```bash
ros2 topic echo /odrive_axis0/controller_status
# controller_status.enabled reflects the enable latch
```

### Enable / disable latch

```bash
# Enable (allows control + closed-loop requests)
ros2 service call /odrive_axis0/set_enabled std_srvs/srv/SetBool "{data: true}"

# Disable (requests IDLE and ignores control_message)
ros2 service call /odrive_axis0/set_enabled std_srvs/srv/SetBool "{data: false}"
```

### Enter closed-loop control

```bash
ros2 service call /odrive_axis0/request_axis_state custom_odrive/srv/AxisState \
  "{axis_requested_state: 8}"
```

Check `success` / `timed_out` in the response. Return to idle with `axis_requested_state: 1`.

### Velocity control (rad/s)

Spin at 2π rad/s (~1 turn/s):

```bash
ros2 topic pub -r 10 /odrive_axis0/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 2, input_mode: 2, input_pos: 0.0, input_vel: 6.283185, input_torque: 0.0}"
```

Stop (0 rad/s), still in velocity mode:

```bash
ros2 topic pub -r 10 /odrive_axis0/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 2, input_mode: 2, input_pos: 0.0, input_vel: 0.0, input_torque: 0.0}"
```

Press `Ctrl+C` to stop the publisher when finished.

### Typical sequence

```bash
ros2 launch custom_odrive example_launch.py
# (other terminal)
ros2 service call /odrive_axis0/set_enabled std_srvs/srv/SetBool "{data: true}"
ros2 service call /odrive_axis0/request_axis_state custom_odrive/srv/AxisState "{axis_requested_state: 8}"
ros2 topic pub -r 10 /odrive_axis0/control_message custom_odrive/msg/ControlMessage \
  "{control_mode: 2, input_mode: 2, input_pos: 0.0, input_vel: 6.283185, input_torque: 0.0}"
# later: set input_vel to 0.0, then request IDLE (1) and/or set_enabled false
```

## Compatible devices (upstream)

- ODrive Pro, ODrive S1, ODrive Micro (not ODrive 3.x)

For ODrive firmware / cyclic message setup, see the [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).
