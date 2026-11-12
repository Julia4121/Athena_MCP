"""Publish a stationary Offboard setpoint using PX4's own boot clock."""

from __future__ import annotations

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node

from sim_bringup.test_support import ControlStream, Px4Time, px4_subscription_qos, resolve_px4_topic


class Heartbeat(Node):
    def __init__(self) -> None:
        super().__init__("athena_offboard_heartbeat")
        self.px4_time = Px4Time()
        self.position: VehicleLocalPosition | None = None
        self.stream: ControlStream | None = None
        qos = px4_subscription_qos()
        status_topic = resolve_px4_topic(self, "/fmu/out/vehicle_status", 20)
        position_topic = resolve_px4_topic(self, "/fmu/out/vehicle_local_position", 20)
        self.create_subscription(VehicleStatus, status_topic, self.px4_time.observe, qos)
        self.create_subscription(VehicleLocalPosition, position_topic, self._on_position, qos)
        self.create_timer(0.2, self._start_when_ready)

    def _on_position(self, message: VehicleLocalPosition) -> None:
        if message.xy_valid and message.z_valid:
            self.position = message

    def _start_when_ready(self) -> None:
        if self.stream is not None or self.position is None:
            return
        try:
            self.px4_time.now_us()
        except RuntimeError:
            return
        mode = OffboardControlMode()
        mode.position = True
        setpoint = TrajectorySetpoint()
        setpoint.position = [self.position.x, self.position.y, self.position.z]
        setpoint.yaw = self.position.heading
        mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", 10)
        setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.stream = ControlStream(
            self, [(mode_pub, mode), (setpoint_pub, setpoint)],
            timestamp_source=self.px4_time.now_us,
        )
        self.get_logger().info("Publishing a stationary PX4-timed Offboard stream.")


def main() -> None:
    rclpy.init()
    node = Heartbeat()
    try:
        rclpy.spin(node)
    finally:
        if node.stream is not None:
            node.stream.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
