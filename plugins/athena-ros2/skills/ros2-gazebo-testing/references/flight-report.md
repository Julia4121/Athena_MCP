# Measured flight report

The runner writes `flight_report.json` automatically after closing an Athena run's
MCAP bag. The sim_bringup package also provides a read-only `flight_report` command
for reports with custom checkpoints. The report reads the bag, contact intervals,
and result file. It reports
measured PX4 state and failsafe transitions, per-topic sample gaps, contact
intervals with nearby position and vehicle state, and any recorded battery and
post-failsafe manual-control samples. It reports observations and does not
decide whether a scenario passed.
The runner's compact summary includes the maximum first-contact speed measured
for a landing-rail-to-floor interval when that telemetry is available.

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

The report's longest_stable_seconds uses bag reception timestamps and breaks a
hold when the drone leaves the supplied position or speed limits, or when
position telemetry has a gap of more than 0.5 seconds. It records the measured
duration and requested minimum separately; the scenario test owns the
assertion. Topic summaries include sample count, first and last timestamps,
and the largest observed gaps. Failsafe flag and navigation/arming changes
include their timestamps. Contact intervals include their time from bag start
and the most recent recorded status and position at each end. Battery readings
and manual-control samples after a recorded PX4 failsafe state are included
when those topics were recorded. Add optional topics with `record_topics` when
the task needs other evidence.

Inspect collision names against the drone SDF: body-to-floor contact is
different from landing gear touching the floor. Use the measured state and
position context, including first-contact speed, to write the task's own touchdown assertion. Store structured
observed values and evidence timestamps in the scenario result.
