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

## Parameters (`custom_odrive`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `node_id` | `0` | ODrive CAN node ID |
| `interface` | `can0` | SocketCAN interface name |
| `axis_idle_on_shutdown` | `false` | Send IDLE on node shutdown |
| `control_message_in_radians` | `false` | Treat command/feedback pos & vel as rad / rad/s (converted to/from turns on the wire) |
| `invert_direction` | `false` | Invert command and feedback direction for this motor |

## Build

Ubuntu with ROS 2 Humble or newer, or use the Dev Containers in `.devcontainer/`:

```bash
cd /path/to/custom-ros-odrive
colcon build --packages-select custom_odrive
source install/setup.bash
```

## Compatible devices (upstream)

- ODrive Pro, ODrive S1, ODrive Micro (not ODrive 3.x)

For ODrive firmware / cyclic message setup, see the [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).
