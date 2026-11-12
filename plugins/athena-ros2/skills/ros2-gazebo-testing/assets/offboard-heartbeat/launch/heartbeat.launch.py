from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package="athena_offboard_heartbeat",
            executable="heartbeat",
            name="athena_offboard_heartbeat",
            parameters=[{"use_sim_time": True}],
            output="screen",
        )
    ])
