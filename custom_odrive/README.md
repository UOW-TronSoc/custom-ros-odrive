# custom_odrive

Standalone per-motor ROS 2 node for controlling an ODrive over CAN.

Derived from the upstream [`odrive_node`](https://github.com/odriverobotics/ros_odrive/tree/main/odrive_node) package (`odrive_can`), with in-tree epoll + SocketCAN from `odrive_base`.

## Interface

### Parameters

* `node_id`: CAN node ID of the device this node attaches to
* `interface`: SocketCAN interface name
* `axis_idle_on_shutdown`: If true, set the ODrive to IDLE when the node shuts down
* `control_message_in_radians`: If true, `input_pos` / `input_vel` and published `pos_estimate` / `vel_estimate` use rad / rad/s (converted to/from turns on the CAN wire)
* `invert_direction`: If true, invert command and feedback direction for this motor

### Subscribes

* `control_message` (`custom_odrive/msg/ControlMessage`): setpoints for the ODrive

### Publishes

* `controller_status` (`custom_odrive/msg/ControllerStatus`): controller-level feedback  
  Publishes when a full cyclic set arrives, or when encoder estimates alone (or heartbeat+encoder) are available.
* `odrive_status` (`custom_odrive/msg/ODriveStatus`): system-level feedback (requires error/temp/bus cyclic messages)

### Services

* `request_axis_state` (`custom_odrive/srv/AxisState`): request an axis state transition
* `clear_errors` (`std_srvs/srv/Empty`): clear errors without changing axis state

## Example

```bash
ros2 launch custom_odrive example_launch.yaml
```

For firmware cyclic-message setup, see the ODrive [ROS CAN Package Guide](https://docs.odriverobotics.com/v/latest/guides/ros-package.html).
