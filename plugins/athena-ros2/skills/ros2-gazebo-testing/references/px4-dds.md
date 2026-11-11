# PX4 1.16 DDS quick reference

For this container, inspect `/opt/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` for the configured topic/type map and the workspace's `px4_msgs/msg/*.msg` for fields and constants. In that YAML, `publications` are PX4 → ROS (`/fmu/out`); `subscriptions` are ROS → PX4 (`/fmu/in`). A nonzero `MESSAGE_VERSION` in a message definition adds `_vN` to its ROS topic name: `VehicleStatus` version 1 appears as `/fmu/out/vehicle_status_v1`.

| Purpose | Topic | Message |
| --- | --- | --- |
| Mode and arming state | `/fmu/out/vehicle_status_v1` | `VehicleStatus` |
| Local position | `/fmu/out/vehicle_local_position` | `VehicleLocalPosition` |
| Failsafe flags | `/fmu/out/failsafe_flags` | `FailsafeFlags` |
| Command acknowledgement | `/fmu/out/vehicle_command_ack` | `VehicleCommandAck` |
| Vehicle command | `/fmu/in/vehicle_command` | `VehicleCommand` |
| Offboard heartbeat and setpoint | `/fmu/in/offboard_control_mode`, `/fmu/in/trajectory_setpoint` | `OffboardControlMode`, `TrajectorySetpoint` |
| Manual input | `/fmu/in/manual_control_input` | `ManualControlSetpoint` |
| Telemetry status input | `/fmu/in/telemetry_status` | `TelemetryStatus` |

Confirm live names and QoS after sourcing ROS and the workspace with `ros2 topic list --no-daemon` and `ros2 topic info --verbose --no-daemon <topic>`. Use `ros2 interface show px4_msgs/msg/<Message>` before relying on fields or enum values. The bundled `sim_bringup.test_support.px4_subscription_qos()` matches PX4 telemetry in this setup. A command acknowledgement confirms receipt; check `VehicleStatus` or measured motion for the requested effect.

With the scaffold, ROS nodes need `use_sim_time=true` to follow Gazebo's `/clock`; XRCE timestamp synchronization is disabled. PX4 `timestamp` fields are microseconds, so verify clock origin and units before comparing them with ROS time. Gazebo world coordinates are ENU; PX4 local coordinates are NED. See the [PX4 1.16 ROS 2 guide](https://docs.px4.io/v1.16/en/ros2/user_guide.html) for the full time and frame conventions.
