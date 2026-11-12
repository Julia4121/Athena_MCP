"""Local MCP server for Athena ROS 2 tools."""

import os
import signal
import subprocess
import uuid
from datetime import datetime, timezone
from pathlib import Path

from mcp.server import MCPServer
from simulation import config
from simulation.runner import ApplicationLaunch, SpawnPose, ros_runtime_environment, run_test
from simulation.world import WorldObject, build_world_file

mcp = MCPServer("athena-ros2")


@mcp.tool()
def dev_exec(command: str, cwd: str = "/home/ws", timeout_seconds: int = 600) -> dict[str, object]:
    """Run an arbitrary development command inside drone-development."""

    if not command.strip():
        raise ValueError("command must not be empty")
    if not 1 <= timeout_seconds <= 3600:
        raise ValueError("timeout_seconds must be between 1 and 3600")
    directory = Path(cwd).resolve()
    if not directory.is_dir():
        raise ValueError(f"cwd does not exist in drone-development: {cwd}")
    logs = Path("/home/ws/agent/athena/commands")
    logs.mkdir(parents=True, exist_ok=True)
    name = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "-" + uuid.uuid4().hex[:8]
    log_path = logs / f"{name}.log"
    shell = (
        "source /opt/ros/jazzy/setup.bash && "
        "{ test ! -f /home/ws/install/setup.bash || source /home/ws/install/setup.bash; } && "
        + command
    )
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            ["bash", "-lc", shell], cwd=directory, stdout=log,
            stderr=subprocess.STDOUT, start_new_session=True,
            env=ros_runtime_environment(),
        )
        timed_out = False
        try:
            exit_code = process.wait(timeout=timeout_seconds)
        except subprocess.TimeoutExpired:
            timed_out = True
            os.killpg(process.pid, signal.SIGTERM)
            try:
                exit_code = process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                exit_code = process.wait()
    with log_path.open("rb") as log:
        log.seek(max(0, log_path.stat().st_size - 32000))
        output_tail = log.read().decode(errors="replace")
    return {
        "exit_code": exit_code,
        "timed_out": timed_out,
        "log_path": str(log_path),
        "output_tail": output_tail,
    }


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
    px4_parameters: dict[str, int | float] | None = None,
    source_repository_path: str | None = None,
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
        px4_parameters,
        source_repository_path,
    )


if __name__ == "__main__":
    mcp.run()
