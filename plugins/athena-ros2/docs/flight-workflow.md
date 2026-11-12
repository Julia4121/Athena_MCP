# Athena ROS 2 flight workflow

## One-time laptop setup

Codex and the plugin run on the laptop. Set `ATHENA_PLATFORM_ROOT` to the `drone_platform` checkout, `ATHENA_ATLAS_ROOT` to the Atlas checkout, and optionally `ATHENA_WORKSPACE` to the host ROS workspace. Defaults are `~/drone_platform`, `~/Projects/agentic.worktrees/atlas`, and `<platform>/.workspaces/development`. The laptop needs Docker and `uv`; private repository creation needs `GITHUB_TOKEN`. The Pi needs SSH access to published package repositories and its normal deployment prerequisites.

Build the shared development/AAA image once, and again after its Dockerfile changes:

```bash
"$ATHENA_PLATFORM_ROOT/containers/development/build_image.sh"
```

The `athena-ros2` MCP entry starts `drone-development` with ROS domain 145 and launches the simulation server inside it. Its plugin mount is read-only; `/home/ws` is the writable host workspace. If an older container lacks that mount, stop it when no work is running and let the MCP entry recreate it. The `athena-aaa` MCP entry runs on the host; the AAA launcher creates a separate `drone-aaa` container with ROS domain 0. Do not run PX4 simulation concurrently with AAA preparation.

## Source repositories and simulation

Prepare the bringup checkout with `python3 <plugin>/repositories.py bringup --workspace "$ATHENA_WORKSPACE"`. It reuses the existing `athena_NN` and seeds `sim_bringup` if needed; a new numbered private repository is created only when none exists. `athena_NN` contains only `sim_bringup` and `atlas_bringup`. The current workspace reuses `athena_01`. Ensure `atlas_bringup` is present before AAA testing.

For a new feature package, run `python3 <plugin>/repositories.py feature --workspace "$ATHENA_WORKSPACE" --package <ros_package_name>`. It uses the current workspace's Git layout: one existing source repository for a monorepo, or a private repository named for the ROS package when most first-party packages have their own checkout. It stops for an unclear mixed layout. Existing packages stay in their current repository.

Write the feature, unit tests, launch file, and simulation assertions in that source repository. Add a feature-specific scenario module and manifest entry to `atlas_bringup` in `athena_NN`; do not put the feature package there. Set the manifest entry's `feature_package` to the deployed ROS package name, and write measurable assertions in the scenario module. Build and debug through `dev_exec`. Commit and publish the feature source before the final simulation run. Call `run_simulation_test` with `source_repository_path=/home/ws/src/<feature-repository>` and keep the resulting `result.json`, logs, and bag. That tool rebuilds the application package, records the source commit before and after the test, and rejects source changes during the run. Rerun after any source change.

Commit and publish the changed `athena_NN` checkout as well. Pin the feature repository's published SHA in `development.repos` and `onboard.repos`, and the `athena_NN` SHA in `development.repos`. Run `drone_platform/scripts/validate_repository_set.py`, then commit and publish the platform main branch. AAA preparation checks that the local bringup checkout is clean and matches the development pin. The Pi builds its own ARM64 workspace; the laptop install tree is not deployed.

## Pi deployment and AAA test

Ask the operator to confirm PX4 is disarmed and all propellers are removed. Call `deploy_onboard(repository_path="src/<feature-repository>", package_name=..., source_sha=..., platform_sha=..., simulation_result_path=..., props_removed_confirmed=true)`, then poll `aaa_status(job_id)`. The host job checks the clean/published commits, the simulation result, Pi Git access, and live disarm; it runs the Pi's `deploy.py ros`, verifies both Pi SHAs and the installed ROS package. If AAA Wi-Fi cannot reach Git, it uses the documented laptop hotspot. On a failed deployment, it leaves the working route available for recovery.

While propellers remain removed, call `check_onboard(deploy_job_id=..., pi_launch_file=..., props_removed_confirmed=true, pi_launch_arguments=[...], expected_topics=[...])` and poll `aaa_status`. The job briefly runs the installed launch on the Pi, checks the expected ROS topics and live disarm state, stops the launch after disarm, and saves its log. Choose topics that show the feature is running. Refit the aircraft for flight only after the operator accepts the test plan and physical setup.

Call `prepare_aaa()`, poll `aaa_status`, and open `http://127.0.0.1:8081/` for operator prompts. Preparation rebuilds the pinned `atlas_bringup`, runs the platform launcher, checks the AAA/Vicon/RC/QGroundControl stack, starts the bridge, and connects the dashboard. Then call `run_aaa_test(deploy_job_id=..., bench_job_id=..., prepare_job_id=..., scenario_id=..., pi_package=..., pi_launch_file=..., pi_launch_arguments=[...])` with the same launch inputs used for the Pi check. It starts the Pi launch only for the test, invokes `atlas-scenario-runner`, and stops the Pi launch after disarm is confirmed. The runner owns fresh dashboard approval, arming supervision, recording, watchdog, landing, and disarm confirmation. If landing or disarm is uncertain, the job retains the live stack and Pi process for the operator.

The final report should name the feature and platform SHAs, the simulation result path, the Pi deployment log, the AAA run directory and bag, the feature assertions, and the observed landing/disarm state. Report a real-drone pass only when the simulator assertions, Pi SHA verification, feature-specific scenario, and landing/disarm checks all pass.
