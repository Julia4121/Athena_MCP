---
name: ros2-aaa-testing
description: Deploy a simulated ROS 2 feature to the Athena Pi and verify it in a supervised AAA Vicon flight. Use after a feature-specific PX4 simulation test passes.
---

# Supervised AAA verification

Read the [flight workflow](../../docs/flight-workflow.md) for the maintained commands, paths, and tool inputs. Use `ros2-development` and `ros2-gazebo-testing` for code and simulation.

- Keep `sim_bringup` and `atlas_bringup` in the existing `athena_NN` repository. For a new feature package, inspect the workspace's first-party Git roots before choosing its repository. The current drone workspace uses a separate, properly named repository for each new feature package; a single-repository workspace keeps the new package in that repository.
- Write the feature's assertions and test launch in its source repository. Put its supervised AAA scenario module and manifest entry in `atlas_bringup`, with `feature_package` set to the deployed package name. The scenario must observe the new behavior and fail when its measurable criteria are not met. A generic hover result does not verify an unrelated feature.
- Publish the feature and platform manifest commits, then run a final simulation against the clean feature commit with `source_repository_path`. `deploy_onboard` requires that matching simulation result. Before deployment, obtain a fresh operator statement that PX4 is disarmed and propellers are removed; pass `props_removed_confirmed=true` only after that statement.
- After deployment, call `check_onboard` with a fresh propeller-removal confirmation and feature-specific expected topics. The job checks the Pi commit, runs the new launch while disarmed, records its log, and stops it after confirmed disarm. Then call `prepare_aaa`. The operator handles placement, Vicon, QGroundControl, RC, and the fresh dashboard flight approval. `run_aaa_test` requires the successful Pi check, starts the same launch for that session, and uses the existing Atlas supervisor. A new control node must wait for the operator's arm and mode actions; it must not arm itself.
- Report real-drone success only with a matching simulation commit, verified Pi commit, feature-specific Atlas result `passed`, `flight_started=true`, and confirmed landing and disarm. If disarm is uncertain, leave the live stack and Pi launch available to the operator and report the uncertainty immediately.
