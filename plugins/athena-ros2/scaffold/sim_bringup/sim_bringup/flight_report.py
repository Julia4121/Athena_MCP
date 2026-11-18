"""Summarize measured PX4 flight evidence from an Athena simulation run."""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
from itertools import pairwise
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class PositionSample:
    wall_time_ns: int
    forward_m: float
    right_m: float
    agl_m: float
    speed_m_s: float


def longest_stable_hold(
    samples: list[PositionSample],
    target: tuple[float, float, float],
    *,
    horizontal_tolerance_m: float,
    vertical_tolerance_m: float,
    maximum_speed_m_s: float,
    start_ns: int,
    end_ns: int,
    maximum_sample_gap_seconds: float = 0.5,
) -> dict[str, float | int | None]:
    """Measure a continuous hold from recorded positions in one time window."""

    best_seconds = 0.0
    best_start: int | None = None
    best_end: int | None = None
    current_start: int | None = None
    previous_ns: int | None = None
    closest_error_m = math.inf
    maximum_gap_ns = int(maximum_sample_gap_seconds * 1_000_000_000)
    for sample in samples:
        if not start_ns <= sample.wall_time_ns <= end_ns:
            continue
        horizontal_error = math.hypot(sample.forward_m - target[0], sample.right_m - target[1])
        vertical_error = abs(sample.agl_m - target[2])
        closest_error_m = min(closest_error_m, math.hypot(horizontal_error, vertical_error))
        stable = (
            horizontal_error <= horizontal_tolerance_m
            and vertical_error <= vertical_tolerance_m
            and sample.speed_m_s <= maximum_speed_m_s
        )
        if not stable:
            current_start = None
            previous_ns = None
            continue
        if previous_ns is None or sample.wall_time_ns - previous_ns > maximum_gap_ns:
            current_start = sample.wall_time_ns
        previous_ns = sample.wall_time_ns
        assert current_start is not None
        seconds = (sample.wall_time_ns - current_start) / 1_000_000_000
        if seconds >= best_seconds:
            best_seconds = seconds
            best_start = current_start
            best_end = sample.wall_time_ns
    return {
        "longest_stable_seconds": round(best_seconds, 3),
        "closest_position_error_m": round(closest_error_m, 3) if math.isfinite(closest_error_m) else None,
        "hold_start_wall_time_ns": best_start,
        "hold_end_wall_time_ns": best_end,
    }


def _topic(topics: dict[str, str], base: str) -> str | None:
    if base in topics:
        return base
    versions = [
        name for name in topics
        if re.fullmatch(re.escape(base) + r"_v\d+", name)
    ]
    return max(versions, key=lambda name: int(name.rsplit("_v", 1)[1])) if versions else None


def _topics_for_base(topics: dict[str, str], base: str) -> list[str]:
    return sorted(
        name for name in topics
        if name == base or re.fullmatch(re.escape(base) + r"_v\d+", name)
    )


def _enum_names(message_type: type, prefix: str) -> dict[int, str]:
    return {
        int(value): name.removeprefix(prefix).lower()
        for name in dir(message_type)
        if name.startswith(prefix)
        and name != f"{prefix}MAX"
        and isinstance((value := getattr(message_type, name)), int)
    }


def _simple_fields(message: Any, *, booleans_only: bool = False) -> dict[str, Any]:
    """Return scalar and scalar-array fields, omitting PX4's separate boot time."""

    fields = message.get_fields_and_field_types()
    values: dict[str, Any] = {}
    for name in fields:
        if name == "timestamp":
            continue
        value = getattr(message, name)
        if isinstance(value, bool):
            if booleans_only:
                values[name] = value
        elif isinstance(value, (int, float, str)) and not booleans_only:
            values[name] = (
                float(value) if isinstance(value, float) and math.isfinite(value) else
                None if isinstance(value, float) else value
            )
        elif not booleans_only and isinstance(value, (list, tuple)) and all(
            isinstance(item, (bool, int, float, str)) for item in value
        ):
            values[name] = [
                float(item) if isinstance(item, float) and math.isfinite(item) else
                None if isinstance(item, float) else item
                for item in value
            ]
    return values


def read_bag(
    run_dir: Path,
) -> tuple[list[PositionSample], list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    """Read flight states and observations from all topics present in the bag."""

    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    bag = run_dir / "bag"
    if not (bag / "metadata.yaml").is_file():
        raise FileNotFoundError(f"Recorded MCAP bag is missing: {bag}")
    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag), storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topics = {item.name: item.type for item in reader.get_all_topics_and_types()}
    names = {
        purpose: _topic(topics, base)
        for purpose, base in {
            "position": "/fmu/out/vehicle_local_position",
            "status": "/fmu/out/vehicle_status",
            "land": "/fmu/out/vehicle_land_detected",
        }.items()
    }
    if names["position"] is None:
        raise ValueError("The bag has no PX4 local position topic.")
    topic_groups = {
        "failsafe_flags": _topics_for_base(topics, "/fmu/out/failsafe_flags"),
        "battery": _topics_for_base(topics, "/fmu/out/battery_status"),
        "manual_control": sorted({
            name
            for base in (
                "/fmu/in/manual_control_input",
                "/fmu/out/manual_control_setpoint",
            )
            for name in _topics_for_base(topics, base)
        }),
    }
    selected_names = {
        name
        for name in names.values()
        if name is not None
    } | {
        name for group in topic_groups.values() for name in group
    }
    selected = {name: get_message(topics[name]) for name in selected_names}
    status_type = selected[names["status"]] if names["status"] else None
    nav_names = _enum_names(status_type, "NAVIGATION_STATE_") if status_type else {}
    arming_names = _enum_names(status_type, "ARMING_STATE_") if status_type else {}
    raw_positions: list[tuple[int, Any]] = []
    statuses: list[dict[str, Any]] = []
    land_states: list[dict[str, Any]] = []
    topic_times: dict[str, list[int]] = {name: [] for name in topics}
    failsafe_flag_samples: list[dict[str, Any]] = []
    battery_samples: dict[str, list[dict[str, Any]]] = {
        name: [] for name in topic_groups["battery"]
    }
    manual_samples: dict[str, list[dict[str, Any]]] = {
        name: [] for name in topic_groups["manual_control"]
    }
    while reader.has_next():
        topic, raw, wall_time_ns = reader.read_next()
        wall_time_ns = int(wall_time_ns)
        topic_times.setdefault(topic, []).append(wall_time_ns)
        message_type = selected.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(raw, message_type)
        if topic == names["position"] and message.xy_valid and message.z_valid:
            raw_positions.append((wall_time_ns, message))
        elif topic == names["status"]:
            statuses.append({
                "wall_time_ns": wall_time_ns,
                "nav_state": int(message.nav_state),
                "nav_state_name": nav_names.get(int(message.nav_state)),
                "arming_state": int(message.arming_state),
                "arming_state_name": arming_names.get(int(message.arming_state)),
                "failsafe": bool(message.failsafe),
            })
        elif topic == names["land"]:
            land_states.append({"wall_time_ns": wall_time_ns, "landed": bool(message.landed)})
        elif topic in topic_groups["failsafe_flags"]:
            failsafe_flag_samples.append({
                "wall_time_ns": wall_time_ns,
                "topic": topic,
                "flags": _simple_fields(message, booleans_only=True),
            })
        elif topic in battery_samples:
            battery_samples[topic].append({
                "wall_time_ns": wall_time_ns,
                "values": _simple_fields(message),
            })
        elif topic in manual_samples:
            manual_samples[topic].append({
                "wall_time_ns": wall_time_ns,
                "values": _simple_fields(message),
            })
    if not raw_positions:
        raise ValueError("The bag has no valid PX4 local position samples.")
    first = raw_positions[0][1]
    result_path = run_dir / "result.json"
    result = json.loads(result_path.read_text(encoding="utf-8")) if result_path.is_file() else {}
    spawn_yaw = result.get("inputs", {}).get("spawn_pose", {}).get("yaw_rad")
    heading = (math.pi / 2 - float(spawn_yaw)) % (2 * math.pi) if spawn_yaw is not None else float(first.heading)
    if not math.isfinite(heading):
        raise ValueError("The first PX4 local position has no finite heading.")
    x0, y0, z0 = float(first.x), float(first.y), float(first.z)
    positions = []
    for wall_time_ns, message in raw_positions:
        dx, dy = float(message.x) - x0, float(message.y) - y0
        vx, vy, vz = float(message.vx), float(message.vy), float(message.vz)
        values = (dx, dy, float(message.z), vx, vy, vz)
        if not all(math.isfinite(value) for value in values):
            continue
        positions.append(PositionSample(
            wall_time_ns=wall_time_ns,
            forward_m=dx * math.cos(heading) + dy * math.sin(heading),
            right_m=-dx * math.sin(heading) + dy * math.cos(heading),
            agl_m=z0 - float(message.z),
            speed_m_s=math.sqrt(vx * vx + vy * vy + vz * vz),
        ))
    observations = {
        "topic_sample_gaps": _topic_gap_summaries(topic_times),
        "failsafe_flag_samples": failsafe_flag_samples,
        "battery_samples": battery_samples,
        "manual_control_samples": manual_samples,
    }
    return positions, statuses, land_states, observations


def _topic_gap_summaries(topic_times: dict[str, list[int]]) -> dict[str, dict[str, Any]]:
    summaries: dict[str, dict[str, Any]] = {}
    for topic, raw_times in sorted(topic_times.items()):
        times = sorted(raw_times)
        if not times:
            continue
        gaps = [
            {"start_wall_time_ns": first, "end_wall_time_ns": second,
             "duration_seconds": round((second - first) / 1_000_000_000, 6)}
            for first, second in pairwise(times)
        ]
        largest = sorted(gaps, key=lambda item: item["duration_seconds"], reverse=True)[:5]
        summaries[topic] = {
            "sample_count": len(times),
            "first_wall_time_ns": times[0],
            "last_wall_time_ns": times[-1],
            "gap_count": len(gaps),
            "maximum_gap_seconds": largest[0]["duration_seconds"] if largest else None,
            "largest_gaps": largest,
        }
    return summaries


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _state_transitions(statuses: list[dict[str, Any]]) -> list[dict[str, Any]]:
    fields = ("nav_state", "arming_state", "failsafe")
    transitions: list[dict[str, Any]] = []
    previous: tuple[Any, ...] | None = None
    for status in statuses:
        state = tuple(status.get(field) for field in fields)
        if state != previous:
            transitions.append({key: status.get(key) for key in (
                "wall_time_ns", *fields, "nav_state_name", "arming_state_name"
            )})
            previous = state
    return transitions


def _flag_transitions(samples: list[dict[str, Any]]) -> list[dict[str, Any]]:
    transitions: list[dict[str, Any]] = []
    previous: dict[str, bool] | None = None
    for sample in samples:
        flags = sample.get("flags", {})
        if not isinstance(flags, dict):
            continue
        changed = (
            dict(flags) if previous is None else {
                name: value for name, value in flags.items()
                if previous.get(name) != value
            }
        )
        if changed:
            transitions.append({
                "wall_time_ns": sample["wall_time_ns"],
                "topic": sample.get("topic"),
                "changed_flags": changed,
                "flags": dict(flags),
            })
        previous = dict(flags)
    return transitions


def _latest_at_or_before(
    samples: list[dict[str, Any]], wall_time_ns: int
) -> dict[str, Any] | None:
    latest = None
    for sample in samples:
        if sample["wall_time_ns"] > wall_time_ns:
            break
        latest = sample
    return latest


def _contact_context(
    contacts: list[dict[str, Any]],
    bag_start_ns: int,
    statuses: list[dict[str, Any]],
    positions: list[PositionSample],
) -> list[dict[str, Any]]:
    contextual: list[dict[str, Any]] = []
    position_records = [
        {
            "wall_time_ns": sample.wall_time_ns,
            "forward_m": sample.forward_m,
            "right_m": sample.right_m,
            "agl_m": sample.agl_m,
            "speed_m_s": sample.speed_m_s,
        }
        for sample in positions
    ]
    for contact in contacts:
        start = int(contact["first_wall_time_ns"])
        end = int(contact["last_wall_time_ns"])
        start_state = _latest_at_or_before(statuses, start)
        end_state = _latest_at_or_before(statuses, end)
        start_position = _latest_at_or_before(position_records, start)
        end_position = _latest_at_or_before(position_records, end)
        contextual.append({
            **contact,
            "time_context": {
                "start_seconds_from_bag_start": round((start - bag_start_ns) / 1e9, 3),
                "end_seconds_from_bag_start": round((end - bag_start_ns) / 1e9, 3),
                "status_at_start": start_state,
                "status_at_end": end_state,
                "position_at_start": start_position,
                "position_at_end": end_position,
            },
        })
    return contextual


def _battery_report(samples: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
    return {
        topic: {
            "sample_count": len(items),
            "first_wall_time_ns": items[0]["wall_time_ns"] if items else None,
            "last_wall_time_ns": items[-1]["wall_time_ns"] if items else None,
            "readings": items,
        }
        for topic, items in samples.items()
        if items
    }


def _manual_after_failsafe(
    statuses: list[dict[str, Any]],
    samples: dict[str, list[dict[str, Any]]],
) -> dict[str, Any]:
    activation_times = [
        item["wall_time_ns"] for item in statuses if item.get("failsafe")
    ]
    activated_ns = min(activation_times) if activation_times else None
    after = {
        topic: {
            "sample_count": sum(
                activated_ns is not None and item["wall_time_ns"] >= activated_ns
                for item in items
            ),
            "samples": [
                item for item in items
                if activated_ns is not None and item["wall_time_ns"] >= activated_ns
            ],
        }
        for topic, items in samples.items()
    }
    return {
        "failsafe_activation_wall_time_ns": activated_ns,
        "topics": after,
    }


def build_report(
    run_dir: Path,
    positions: list[PositionSample],
    statuses: list[dict[str, Any]],
    land_states: list[dict[str, Any]],
    checkpoints: list[dict[str, Any]],
    events: list[dict[str, Any]],
    observations: dict[str, Any] | None = None,
) -> dict[str, Any]:
    if not positions:
        raise ValueError("At least one position sample is required.")
    completion_times = {
        str(item["step"]): int(item["wall_time_ns"])
        for item in events
        if item.get("event") == "step_completed"
        and isinstance(item.get("step"), str)
        and isinstance(item.get("wall_time_ns"), int)
    }
    previous_end = positions[0].wall_time_ns
    measured = []
    for checkpoint in checkpoints:
        name = str(checkpoint["name"])
        target = tuple(float(value) for value in checkpoint["target_forward_right_agl_m"])
        if len(target) != 3 or not all(math.isfinite(value) for value in target):
            raise ValueError(f"{name}: target must contain three finite coordinates.")
        minimum_seconds = float(checkpoint["minimum_seconds"])
        horizontal_tolerance = float(checkpoint["horizontal_tolerance_m"])
        vertical_tolerance = float(checkpoint["vertical_tolerance_m"])
        maximum_speed = float(checkpoint["maximum_speed_m_s"])
        if not all(math.isfinite(value) and value >= 0 for value in (
            minimum_seconds, horizontal_tolerance, vertical_tolerance, maximum_speed
        )):
            raise ValueError(f"{name}: duration, tolerances, and speed must be finite and nonnegative.")
        step = checkpoint.get("step")
        if step is not None:
            if step not in completion_times:
                raise ValueError(f"{name}: step_completed event {step!r} is missing.")
            end_ns = completion_times[step]
            start_ns = previous_end
            if end_ns < start_ns:
                raise ValueError(f"{name}: step events are not in checkpoint order.")
            previous_end = end_ns
        else:
            start_ns, end_ns = positions[0].wall_time_ns, positions[-1].wall_time_ns
        hold = longest_stable_hold(
            positions, target,
            horizontal_tolerance_m=horizontal_tolerance,
            vertical_tolerance_m=vertical_tolerance,
            maximum_speed_m_s=maximum_speed,
            start_ns=start_ns, end_ns=end_ns,
        )
        measured.append({
            "name": name,
            "target_forward_right_agl_m": target,
            "minimum_seconds": minimum_seconds,
            "window_start_wall_time_ns": start_ns,
            "window_end_wall_time_ns": end_ns,
            **hold,
        })
    result_path = run_dir / "result.json"
    run_result = json.loads(result_path.read_text(encoding="utf-8")) if result_path.is_file() else {}
    contacts_path = run_dir / "contacts.jsonl"
    contacts = _read_jsonl(contacts_path)
    observations = observations or {}
    failsafe_flag_samples = observations.get("failsafe_flag_samples", [])
    battery_samples = observations.get("battery_samples", {})
    manual_samples = observations.get("manual_control_samples", {})
    recorded_starts = [
        item["first_wall_time_ns"]
        for item in observations.get("topic_sample_gaps", {}).values()
        if isinstance(item.get("first_wall_time_ns"), int)
    ]
    bag_start_ns = min(recorded_starts) if recorded_starts else min(
        sample.wall_time_ns for sample in positions
    )
    return {
        "run_id": run_result.get("run_id", run_dir.name),
        "last_phase": run_result.get("last_phase"),
        "position_samples": len(positions),
        "status_samples": len(statuses),
        "land_state_samples": len(land_states),
        "nav_states_observed": sorted({item["nav_state"] for item in statuses}),
        "failsafe_observed": any(item["failsafe"] for item in statuses),
        "state_transitions": _state_transitions(statuses),
        "failsafe_flag_samples": len(failsafe_flag_samples),
        "failsafe_flag_transitions": _flag_transitions(failsafe_flag_samples),
        "final_nav_state": statuses[-1]["nav_state"] if statuses else None,
        "final_arming_state": statuses[-1]["arming_state"] if statuses else None,
        "final_landed": land_states[-1]["landed"] if land_states else None,
        "topic_sample_gaps": observations.get("topic_sample_gaps", {}),
        "battery": _battery_report(battery_samples),
        "manual_control_after_failsafe": _manual_after_failsafe(statuses, manual_samples),
        "checkpoints": measured,
        "contacts_file_present": contacts_path.is_file(),
        "contacts": _contact_context(contacts, bag_start_ns, statuses, positions),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--checkpoints", type=Path, help="JSON file with a checkpoints list")
    parser.add_argument("--events", type=Path, help="Optional scenario_events.jsonl for ordered step windows")
    parser.add_argument("--output", type=Path, help="Write the JSON report here; otherwise print it")
    args = parser.parse_args()
    plan = json.loads(args.checkpoints.read_text(encoding="utf-8")) if args.checkpoints else {}
    positions, statuses, land_states, observations = read_bag(args.run_dir)
    report = build_report(
        args.run_dir, positions, statuses, land_states,
        plan.get("checkpoints", []),
        _read_jsonl(args.events) if args.events else [],
        observations,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
