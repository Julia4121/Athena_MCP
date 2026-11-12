# Athena ROS 2 for Codex

This repository packages ROS 2 development and Gazebo testing skills with a local MCP server as a Codex plugin. The [development skill](plugins/athena-ros2/skills/ros2-development/SKILL.md) contains the [Henki ROS 2 Best Practices](https://github.com/henki-robotics/henki_ros2_best_practices) directly, with attribution and the upstream license. The [simulation skill](plugins/athena-ros2/skills/ros2-gazebo-testing/SKILL.md) guides Codex through writing and testing a feature in Gazebo.

## Install

You need access to this private GitHub repository, [Codex](https://developers.openai.com/codex), Python 3.11 or newer, and [`uv`](https://docs.astral.sh/uv/) on your `PATH`. `build_world` and `run_simulation_test` run in the same container as Codex. Simulation requires ROS 2 Jazzy, Gazebo Harmonic, PX4 1.16 SITL, the Micro XRCE-DDS agent, `ros_gz_bridge`, and `colcon`; build `px4_msgs` in the ROS workspace before running a PX4 test. The existing `sim_agent` Docker image provides the simulator stack, but its LangGraph workflow is not used.

```bash
codex plugin marketplace add ssh://git@github.com/Julia4121/Athena_MCP.git
codex plugin add athena-ros2@athena-mcp
```

Restart Codex after installation. The `ros2-development` and `ros2-gazebo-testing` skills can be selected explicitly and are also discoverable during relevant work. The `athena-ros2` MCP server exposes exactly two tools:

- `build_world` adds pillars, tables, chairs, and cardboard boxes to a copy of an empty basement or tripleA room SDF. The output must be a distinct path inside the ROS workspace, where it can remain with the regression test.
- `run_simulation_test` starts a supplied X500-compatible drone and world named `environment`, then a ROS application launch file and a Codex-authored test command. Build the ROS workspace with `colcon` before calling it. Optional `px4_parameters` are applied at startup and logged with observed values. The tool starts an MCAP recorder before the application and test, records a small default PX4 evidence set and drone contact intervals, and returns readiness, exit codes, timeouts, and artifact paths under `<workspace>/agent/athena/simulation/`. A one-second quiet gap starts a new interval for a collision pair, separating takeoff from landing. Use `record_topics` to add feature-specific topics; battery is not recorded by default.

The runner reports observations rather than diagnosing the root cause of a failed scenario. Codex uses its authored assertions, logs, and telemetry to decide whether the ROS implementation, scenario, or test needs repair.

The bundled `sim_bringup.test_support` Python module provides PX4 topic discovery, message waits, command acknowledgements, and a pausable control-message stream for Codex-authored tests. It contains no route or assertion logic. A command acknowledgement confirms receipt, so tests should also check the resulting vehicle state and motion.

The input SDFs stay unchanged. When both the room and drone load Gazebo's Sensors system, the runner records a drone copy with the duplicate declaration removed. PX4's XRCE time synchronization is disabled for SITL startup; tests should use `/clock` and check PX4 timestamp semantics explicitly.

The self-hosted configurator can create a missing X500 drone SDF at `drone.html` and an empty room SDF at `environment.html`. The plugin bundles the resource models referenced by those SDFs; see [model attribution](plugins/athena-ros2/THIRD_PARTY_MODELS.md).

The separate `sim_evaluation_pipeline` project can run `codex-athena` alongside `sim_agent` and plain `codex`. It installs a pinned snapshot of this plugin inside an isolated Codex container and scores Athena's recorded bags, contacts, and PX4 logs with the same telemetry checks used for the other runners. The benchmark prompt leaves Codex free to choose when its two tools help.

To update an existing installation, run `codex plugin marketplace upgrade athena-mcp` followed by `codex plugin add athena-ros2@athena-mcp`, then restart Codex.

## Local development

From this repository, install the dependencies and check the lock file with:

```bash
uv sync --project plugins/athena-ros2 --locked
```

The plugin is defined under `plugins/athena-ros2`; `.agents/plugins/marketplace.json` makes it installable from this repository. The simulation server code lives in `simulation/`, and its fixed ROS bringup package lives in `scaffold/sim_bringup/`.
