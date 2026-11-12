# Measured flight report

The sim_bringup package provides a read-only flight_report command. It reads
an Athena run's MCAP bag, contact intervals, and result file. It reports
measured PX4 modes, failsafe, final arming and landing state, and every
recorded collision pair. It does not assign a root cause.

For hover measurements, write a small JSON checkpoint file based on the
requested scenario. Coordinates are launch-facing forward, right, and
height above the starting PX4 position, in metres. Tolerances and maximum
speed are chosen for that feature; they are not fixed by Athena.

    {
      "checkpoints": [
        {
          "name": "first hover",
          "step": "first_hover",
          "target_forward_right_agl_m": [0, 0, 2],
          "minimum_seconds": 3,
          "horizontal_tolerance_m": 0.4,
          "vertical_tolerance_m": 0.3,
          "maximum_speed_m_s": 0.25
        }
      ]
    }

If the test writes step_completed events with step and wall_time_ns in a
scenario_events.jsonl file, pass it with --events. Each checkpoint is then
measured between the previous checkpoint's completion and its own completion,
so a return to the same location cannot reuse an earlier hover. Without
events, each target is searched over the full bag; use that only for a
one-off diagnostic or a unique location.

    source /opt/ros/jazzy/setup.bash
    source /workspace/install/setup.bash
    ros2 run sim_bringup flight_report /workspace/agent/athena/simulation/RUN_ID \
      --checkpoints /workspace/checkpoints.json \
      --events /workspace/scenario_events.jsonl \
      --output /workspace/agent/athena/simulation/RUN_ID/flight_report.json

The report's longest_stable_seconds uses bag reception timestamps and
breaks a hold when the drone leaves the declared position or speed limits,
or when telemetry has a gap of more than 0.5 seconds. Compare the result
with the requested duration and keep a modest margin in the controller.
Inspect collision names against the drone SDF: body-to-floor contact is
different from landing gear touching the floor. Confirm the vehicle has
disarmed and that the observed landing state matches the flight evidence.
