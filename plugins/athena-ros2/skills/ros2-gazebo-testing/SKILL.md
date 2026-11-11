---
name: ros2-gazebo-testing
description: Design and run Gazebo/PX4 integration tests for ROS 2 drone features using supplied X500-compatible drone and supported room SDFs. Use when a feature needs simulation evidence, not for ordinary unit tests.
---

# ROS 2 Gazebo testing

Use this skill alongside `ros2-development` when a ROS 2 feature needs a simulated flight or sensor test. You write the ROS code, application launch file, and test assertions. The Athena MCP tools only build a world and run your test in a supervised simulator.

1. Inspect the feature request, ROS workspace, drone SDF, and empty-room SDF. If an SDF is missing, ask the user to create it with the available drone or environment configurator. This version expects the X500 PX4 airframe and a basement or tripleA configurator room.
2. State what observable behavior would pass and fail. Run a simple baseline before the feature-specific scenario so simulator trouble does not masquerade as a feature failure. Add only pillars, tables, chairs, or cardboard boxes with `build_world`; choose positions that leave a valid drone spawn and route. Its output SDF belongs in the workspace beside the regression test.
3. Write a ROS integration test that connects to the running stack, waits for required nodes, topics, services, or actions with deadlines, and asserts measured behavior. Inspect clock source, QoS, units, and frames. Gazebo world coordinates are ENU; PX4 local coordinates are NED. PX4 1.16 may expose a versioned topic such as `/fmu/out/vehicle_status_v1`; discover the actual topic and use compatible QoS. The scaffold disables XRCE time synchronization in SITL, so do not assume PX4 timestamps are ROS clock timestamps. Avoid fixed sleeps as a substitute for readiness checks.
4. Build the workspace with its normal `colcon` commands. Use or create an application launch file that starts the implementation, then call `run_simulation_test` with that launch entry point and the test command as an argument list. The tool starts Gazebo, PX4, the XRCE agent, and standard sensor bridges before launching the application and test.
5. Read the returned phase, readiness, process exits, test exit code, logs, and requested bag topics. A failed flight can come from the ROS implementation, scenario design, assertions, or simulator setup. Assign a cause only when the evidence supports it; add a focused diagnostic and rerun if needed. Repair the relevant source or test, then repeat the same scenario.

Keep successful test code, application launch changes, and generated world SDF in the ROS workspace. Report the assertion evidence and any remaining uncertainty. Simulator startup by itself does not establish feature correctness.
