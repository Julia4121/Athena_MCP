"""Summarize measured PX4 flight evidence from an Athena simulation run."""

from __future__ import annotations

import argparse
import json
import math
import re
from dataclasses import dataclass
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


def read_bag(run_dir: Path) -> tuple[list[PositionSample], list[dict[str, Any]], list[dict[str, Any]]]:
    """Read only the PX4 topics needed for a compact flight report."""

    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

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
    selected = {name: get_message(topics[name]) for name in names.values() if name is not None}
    raw_positions: list[tuple[int, Any]] = []
    statuses: list[dict[str, Any]] = []
    land_states: list[dict[str, Any]] = []
    while reader.has_next():
        topic, raw, wall_time_ns = reader.read_next()
        message_type = selected.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(raw, message_type)
        if topic == names["position"] and message.xy_valid and message.z_valid:
            raw_positions.append((int(wall_time_ns), message))
        elif topic == names["status"]:
            statuses.append({
                "wall_time_ns": int(wall_time_ns),
                "nav_state": int(message.nav_state),
                "arming_state": int(message.arming_state),
                "failsafe": bool(message.failsafe),
            })
        elif topic == names["land"]:
            land_states.append({"wall_time_ns": int(wall_time_ns), "landed": bool(message.landed)})
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
    return positions, statuses, land_states


def _read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.is_file():
        return []
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def build_report(
    run_dir: Path,
    positions: list[PositionSample],
    statuses: list[dict[str, Any]],
    land_states: list[dict[str, Any]],
    checkpoints: list[dict[str, Any]],
    events: list[dict[str, Any]],
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
            "meets_minimum": hold["longest_stable_seconds"] >= minimum_seconds,
            "window_start_wall_time_ns": start_ns,
            "window_end_wall_time_ns": end_ns,
            **hold,
        })
    result_path = run_dir / "result.json"
    run_result = json.loads(result_path.read_text(encoding="utf-8")) if result_path.is_file() else {}
    contacts_path = run_dir / "contacts.jsonl"
    return {
        "run_id": run_result.get("run_id", run_dir.name),
        "last_phase": run_result.get("last_phase"),
        "position_samples": len(positions),
        "status_samples": len(statuses),
        "land_state_samples": len(land_states),
        "nav_states_observed": sorted({item["nav_state"] for item in statuses}),
        "failsafe_observed": any(item["failsafe"] for item in statuses),
        "final_nav_state": statuses[-1]["nav_state"] if statuses else None,
        "final_arming_state": statuses[-1]["arming_state"] if statuses else None,
        "final_landed": land_states[-1]["landed"] if land_states else None,
        "checkpoints": measured,
        "contacts_file_present": contacts_path.is_file(),
        "contacts": _read_jsonl(contacts_path),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--checkpoints", type=Path, help="JSON file with a checkpoints list")
    parser.add_argument("--events", type=Path, help="Optional scenario_events.jsonl for ordered step windows")
    parser.add_argument("--output", type=Path, help="Write the JSON report here; otherwise print it")
    args = parser.parse_args()
    plan = json.loads(args.checkpoints.read_text(encoding="utf-8")) if args.checkpoints else {}
    positions, statuses, land_states = read_bag(args.run_dir)
    report = build_report(
        args.run_dir, positions, statuses, land_states,
        plan.get("checkpoints", []), _read_jsonl(args.events) if args.events else [],
    )
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
