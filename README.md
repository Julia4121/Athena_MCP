# Athena ROS 2 for Codex

This repository packages ROS 2 development guidance, PX4/Gazebo simulation tools, and a host-side path to supervised AAA/Vicon testing as a Codex plugin. The [flight workflow](plugins/athena-ros2/docs/flight-workflow.md) is the operator and Codex runbook.

## Install

Codex runs on the laptop. Install [Codex](https://developers.openai.com/codex), Python 3.11+, `uv`, Docker, the `drone_platform` checkout, and the Atlas checkout. Build the shared ROS development image using `drone_platform/containers/development/build_image.sh`. Set `ATHENA_PLATFORM_ROOT` and `ATHENA_ATLAS_ROOT` when the checkouts are outside their default paths; set `ATHENA_WORKSPACE` if the ROS workspace is elsewhere.

```bash
codex plugin marketplace add ssh://git@github.com/Julia4121/Athena_MCP.git
codex plugin add athena-ros2@athena-mcp
```

Restart Codex after installation. The plugin has two MCP entries:

- `athena-ros2` starts inside `drone-development`. `dev_exec` runs arbitrary ROS build and debugging commands in that container. `build_world` and `run_simulation_test` run Gazebo, PX4 SITL, and ROS there and store evidence under `/home/ws/agent/athena/simulation/`.
- `athena-aaa` runs on the laptop host. `deploy_onboard`, `check_onboard`, `prepare_aaa`, and `run_aaa_test` start inspectable jobs; `aaa_status` reads their progress and artifacts. The physical flight path uses the maintained `drone_platform` launcher and Atlas runner/dashboard.

The [ROS development skill](plugins/athena-ros2/skills/ros2-development/SKILL.md), [simulation skill](plugins/athena-ros2/skills/ros2-gazebo-testing/SKILL.md), and [AAA skill](plugins/athena-ros2/skills/ros2-aaa-testing/SKILL.md) guide the full sequence. The repository helper keeps `sim_bringup` and `atlas_bringup` in `athena_NN`, while new feature packages follow the workspace's Git layout. The current workspace uses a separate, properly named repository for a new feature package.

## Simulation behavior

`build_world` adds pillars, tables, chairs, and boxes to a copy of a supported empty room SDF in the ROS workspace. `run_simulation_test` starts an X500-compatible drone, PX4 SITL, Gazebo, a ROS application launch, and a Codex-authored test command. It rebuilds the application package before the run and records its source commit before and after when `source_repository_path` is supplied. It records MCAP, startup readiness, process exits, contact intervals, and logs under `/home/ws/agent/athena/simulation/`. Optional PX4 parameters and feature topics are included in the evidence. The runner reports observed facts; the test command supplies the feature assertions.

The copied `sim_bringup.test_support` module provides PX4 topic discovery, message waits, command acknowledgements, and a pausable control-message stream. An acknowledgement confirms receipt, so tests should check the resulting vehicle state and motion. The input SDFs stay unchanged; the runner writes a normalized drone copy if both SDFs load Gazebo's Sensors system. Tests should use `/clock` and check PX4 timestamp semantics. The plugin bundles resource models for SDFs produced by the self-hosted configurator; see [model attribution](plugins/athena-ros2/THIRD_PARTY_MODELS.md).

The separate `sim_evaluation_pipeline` project can compare Athena against `sim_agent` and plain Codex using recorded bags, contacts, and PX4 logs. It snapshots a clean Athena revision for each batch. In that benchmark, Codex and the Athena simulation MCP server run directly inside the simulator container. The AAA server, AAA skill, and `dev_exec` tool are disabled; the bundled `sim_bringup` source is copied into each Athena case workspace. Set `SIM_EVAL_ATHENA_PROJECT` if this checkout is not beside the pipeline. The benchmark also needs the `sim_agent` Python environment, Codex CLI and Code Mode host, and Docker access on the host. From `sim_evaluation_pipeline`:

```bash
../sim_agent/.venv/bin/python run_batch.py --case 11
../sim_agent/.venv/bin/python run_batch.py --case 11 --runner codex-athena
```

The first command runs `sim_agent`, `codex`, and `codex-athena` in that order. Results and pinned source and image records are saved under `~/Projects/storage/sim_eval_artifacts/run_XX/`. The shared scorer checks recorded flight telemetry, world geometry, the drone model, and gear-to-floor contact using collision names and times; it does not use runner-specific contact labels.

## Local checks

```bash
cd plugins/athena-ros2
uv sync --locked
uv run --locked python -m unittest tests.test_repositories tests.test_aaa_workflow -v
```

The plugin source is under `plugins/athena-ros2`. The simulation SDFs stay unchanged; the runner records readiness, process exits, logs, contact intervals, and MCAP data. The [model attribution](plugins/athena-ros2/THIRD_PARTY_MODELS.md) covers bundled Gazebo assets.
