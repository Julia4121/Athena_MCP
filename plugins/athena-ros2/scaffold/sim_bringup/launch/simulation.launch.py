"""Launch the fixed simulator supervisor with per-run SDF paths."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names = (
        "world_sdf",
        "drone_sdf",
        "spawn_x",
        "spawn_y",
        "spawn_z",
        "spawn_yaw",
        "log_dir",
        "status_file",
        "resource_root",
        "px4_parameters",
    )
    arguments = [DeclareLaunchArgument(name) for name in names]
    command = []
    for name in names:
        command.extend(("--" + name.replace("_", "-"), LaunchConfiguration(name)))
    return LaunchDescription(
        [
            *arguments,
            Node(
                package="sim_bringup",
                executable="startup",
                name="sim_bringup",
                output="screen",
                arguments=command,
            ),
        ]
    )
