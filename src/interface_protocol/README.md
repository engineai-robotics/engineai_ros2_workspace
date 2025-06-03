# Interface Protocol

Description of the interface protocol.

## Interface Protocol

| Topic/Service Name | Type | Rate | Message | Description |
| --- | --- | --- | --- | --- |
| /hardware/gamepad_keys | Topic Publish | >=500hz | GamepadKeys.msg | Gamepad Message Feedback |
| /hardware/imu_info | Topic Publish | >=500hz | IMUInfo.msg | IMU Sensor Feedback |
| /hardware/joint_state | Topic Publish | >=500hz | JointState.msg | Joint Status Feedback |
| /hardware/joint_command | Topic Subscription | 0~500hz | JointCommand.msg | Joint Command Subscription |
| /motion/motion_state | Topic Publish | >=5hz | MotionState.msg | Motion Status Feedback of Robot |
| /hardware/motor_debug | Topic Publish | >=50hz | MotorDebug.msg | Feedback of Temperature, Tau, etc. |
| /motion/body_vel_cmd | Topic Publish | >=5hz | BodyVelCmd.msg | body velocity command publish, linear_x_vel ranges [-0.5m/s +0.5m/s] linear_y_vel ranges [-0.2m/s, +0.2m/s]， yaw_vel rangs [-0.5rad/s, 0.5rad/s]|
