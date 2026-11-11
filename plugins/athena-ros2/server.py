"""Local MCP server for Athena ROS 2 tools."""

from mcp.server import MCPServer

from simulation import config
from simulation.runner import ApplicationLaunch, SpawnPose, run_test
from simulation.world import WorldObject, build_world_file


mcp = MCPServer("athena-ros2")


@mcp.tool()
def build_world(
    workspace_path: str,
    base_world_sdf: str,
    output_world_sdf: str,
    objects: list[WorldObject],
) -> dict[str, object]:
    """Add pillars, tables, chairs, and boxes to a copy of an empty supported room SDF."""

    return build_world_file(workspace_path, base_world_sdf, output_world_sdf, objects)


@mcp.tool()
def run_simulation_test(
    workspace_path: str,
    drone_sdf: str,
    world_sdf: str,
    spawn_pose: SpawnPose,
    application_launch: ApplicationLaunch,
    test_argv: list[str],
    timeout_seconds: int = config.TEST_TIMEOUT_SECONDS,
    record_topics: list[str] | None = None,
) -> dict[str, object]:
    """Run a Codex-authored test with Gazebo, PX4, and ROS; return observed evidence."""

    return run_test(
        workspace_path,
        drone_sdf,
        world_sdf,
        spawn_pose,
        application_launch,
        test_argv,
        timeout_seconds,
        record_topics,
    )


if __name__ == "__main__":
    mcp.run()
