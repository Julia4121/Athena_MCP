"""Host-side MCP entry point for supervised AAA operations."""

from mcp.server import MCPServer

from aaa_workflow import read_job, start_job

mcp = MCPServer("athena-aaa")


@mcp.tool()
def deploy_onboard(
    repository_path: str,
    package_name: str,
    source_sha: str,
    platform_sha: str,
    simulation_result_path: str,
    props_removed_confirmed: bool,
) -> dict[str, str]:
    """Deploy a published ROS source commit to the disarmed Pi."""

    return start_job("deploy", locals())


@mcp.tool()
def check_onboard(
    deploy_job_id: str,
    pi_launch_file: str,
    props_removed_confirmed: bool,
    pi_launch_arguments: list[str] | None = None,
    expected_topics: list[str] | None = None,
    duration_seconds: int = 10,
) -> dict[str, str]:
    """Briefly run the deployed launch on a propellers-off, disarmed Pi."""

    return start_job(
        "bench",
        {
            "deploy_job_id": deploy_job_id,
            "pi_launch_file": pi_launch_file,
            "props_removed_confirmed": props_removed_confirmed,
            "pi_launch_arguments": pi_launch_arguments or [],
            "expected_topics": expected_topics or [],
            "duration_seconds": duration_seconds,
        },
    )


@mcp.tool()
def prepare_aaa() -> dict[str, str]:
    """Start the Atlas dashboard, AAA launcher, and supervised bridge."""

    return start_job("prepare", {})


@mcp.tool()
def run_aaa_test(
    deploy_job_id: str,
    bench_job_id: str,
    prepare_job_id: str,
    scenario_id: str,
    pi_package: str,
    pi_launch_file: str,
    pi_launch_arguments: list[str] | None = None,
) -> dict[str, str]:
    """Run a feature-specific Atlas scenario with fresh operator approval."""

    return start_job(
        "flight",
        {
            "deploy_job_id": deploy_job_id,
            "bench_job_id": bench_job_id,
            "prepare_job_id": prepare_job_id,
            "scenario_id": scenario_id,
            "pi_package": pi_package,
            "pi_launch_file": pi_launch_file,
            "pi_launch_arguments": pi_launch_arguments or [],
        },
    )


@mcp.tool()
def aaa_status(job_id: str) -> dict[str, object]:
    """Return a deployment, preparation, or flight job's current evidence."""

    return read_job(job_id)


if __name__ == "__main__":
    mcp.run()
