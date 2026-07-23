# ODrive S1 — wheel_bl (back left)
#
# Used by: ros2 run custom_odrive commission -- --config <this file> ...
# SERIAL_NUMBER is the only identity source for commission.py (no CLI --serial).
# `odrv`, `math`, and enums (MotorType, ControlMode, …) are injected at apply time.
#
# Keep node_id in sync with launch (example_multi_launch: wheel_bl → node_id 2).

SERIAL_NUMBER = "396934453331"

odrv.config.dc_bus_overvoltage_trip_level = 36
odrv.config.dc_bus_undervoltage_trip_level = 21
odrv.config.dc_max_positive_current = math.inf
odrv.config.dc_max_negative_current = -math.inf
odrv.config.brake_resistor0.enable = True
odrv.config.brake_resistor0.resistance = 2.4
odrv.axis0.config.motor.motor_type = MotorType.PMSM_CURRENT_CONTROL
odrv.axis0.config.motor.pole_pairs = 20
odrv.axis0.config.motor.torque_constant = 0.0827
odrv.axis0.config.motor.current_soft_max = 50
odrv.axis0.config.motor.current_hard_max = 70
odrv.axis0.config.motor.calibration_current = 10
odrv.axis0.config.motor.resistance_calib_max_voltage = 2
odrv.axis0.config.calibration_lockin.current = 10
odrv.axis0.motor.motor_thermistor.config.enabled = False
odrv.axis0.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
odrv.axis0.controller.config.input_mode = InputMode.VEL_RAMP
odrv.axis0.controller.config.vel_limit = 22
odrv.axis0.controller.config.vel_limit_tolerance = 1.1363636363636365
odrv.axis0.config.torque_soft_min = -math.inf
odrv.axis0.config.torque_soft_max = math.inf
odrv.axis0.trap_traj.config.accel_limit = 40
odrv.axis0.controller.config.vel_ramp_rate = 40
odrv.can.config.protocol = Protocol.SIMPLE
odrv.can.config.baud_rate = 0
odrv.axis0.config.can.node_id = 2
odrv.axis0.config.can.heartbeat_msg_rate_ms = 10
odrv.axis0.config.can.encoder_msg_rate_ms = 10
odrv.axis0.config.can.iq_msg_rate_ms = 10
odrv.axis0.config.can.torques_msg_rate_ms = 10
odrv.axis0.config.can.error_msg_rate_ms = 100
odrv.axis0.config.can.temperature_msg_rate_ms = 100
odrv.axis0.config.can.bus_voltage_msg_rate_ms = 100
odrv.axis0.config.enable_watchdog = True
odrv.axis0.config.watchdog_timeout = 2
odrv.axis0.config.load_encoder = EncoderId.ONBOARD_ENCODER0
odrv.axis0.config.commutation_encoder = EncoderId.ONBOARD_ENCODER0
odrv.config.enable_uart_a = False
