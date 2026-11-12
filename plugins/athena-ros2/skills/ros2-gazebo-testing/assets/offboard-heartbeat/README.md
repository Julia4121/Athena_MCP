# Offboard heartbeat reference

Copy this small ROS package into the workspace's src directory and build it
with colcon. Pass package athena_offboard_heartbeat and file
heartbeat.launch.py as application_launch to run_simulation_test.

The node waits for PX4 telemetry, holds the current local position as its
setpoint, and publishes an Offboard heartbeat at 20 Hz. It deliberately does
not arm, change flight mode, or fly a mission. Adapt the control stream and
write separate test assertions for the requested feature. A VehicleCommand
can use send_vehicle_command with timestamp_source=node.px4_time.now_us.

The test/heartbeat_test.py script is a small baseline test_argv. It verifies
that the application publishes a heartbeat with a timestamp close to live
PX4 telemetry.
