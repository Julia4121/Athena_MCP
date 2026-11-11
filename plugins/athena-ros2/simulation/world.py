"""Create a test scene by adding known objects to a known empty room SDF."""

from __future__ import annotations

import math
import os
import re
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Literal

from pydantic import BaseModel, Field, field_validator

from simulation import config


class WorldObject(BaseModel):
    """One object placed on the room floor, in Gazebo world coordinates."""

    type: Literal["pillar", "table", "chair", "cardboard_box"]
    x_m: float
    y_m: float
    yaw_rad: float = 0.0
    name: str | None = None
    shape: Literal["circular", "square"] | None = None
    radius_m: float | None = Field(default=None, gt=0)
    width_m: float | None = Field(default=None, gt=0)
    depth_m: float | None = Field(default=None, gt=0)

    @field_validator("x_m", "y_m", "yaw_rad", "radius_m", "width_m", "depth_m")
    @classmethod
    def finite(cls, value: float | None) -> float | None:
        if value is not None and not math.isfinite(value):
            raise ValueError("coordinates and yaw must be finite")
        return value


def _room_bounds(world: ET.Element) -> tuple[float, float, float, float, float, float]:
    room = world.find("model[@name='room']")
    if room is None:
        raise ValueError("The world needs the configurator's room model.")
    limits: dict[str, float] = {}
    for collision in room.findall(".//collision"):
        name = (collision.get("name") or "").removesuffix("_collision")
        if name not in {"floor", "ceiling", "wall_x_min", "wall_x_max", "wall_y_min", "wall_y_max"}:
            continue
        size_text = collision.findtext("geometry/box/size")
        if not size_text:
            continue
        try:
            size = [float(part) for part in size_text.split()]
            pose = [float(part) for part in (collision.findtext("pose") or "").split()]
        except ValueError as exc:
            raise ValueError(f"Room collision {name!r} has invalid geometry.") from exc
        if len(size) != 3 or len(pose) < 3:
            raise ValueError(f"Room collision {name!r} needs a box and pose.")
        x, y, z = pose[:3]
        sx, sy, sz = size
        if name == "floor":
            limits["z_min"] = z + sz / 2
        elif name == "ceiling":
            limits["z_max"] = z - sz / 2
        elif name == "wall_x_min":
            limits["x_min"] = x + sx / 2
        elif name == "wall_x_max":
            limits["x_max"] = x - sx / 2
        elif name == "wall_y_min":
            limits["y_min"] = y + sy / 2
        elif name == "wall_y_max":
            limits["y_max"] = y - sy / 2
    required = {"x_min", "x_max", "y_min", "y_max", "z_min", "z_max"}
    if set(limits) != required:
        raise ValueError("The room needs named floor, ceiling, and four wall box collisions.")
    bounds = tuple(limits[key] for key in ("x_min", "x_max", "y_min", "y_max", "z_min", "z_max"))
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    dimensions = (round(x_max - x_min, 3), round(y_max - y_min, 3), round(z_max - z_min, 3))
    if dimensions not in {(30.0, 2.5, 3.5), (34.0, 15.5, 10.8)}:
        raise ValueError("Only the configurator's basement and tripleA rooms are supported.")
    return bounds


def _footprint(item: WorldObject) -> tuple[str, float, float]:
    if item.type != "pillar":
        if any(value is not None for value in (item.shape, item.radius_m, item.width_m, item.depth_m)):
            raise ValueError(f"{item.type} uses its model's fixed dimensions.")
        width, depth = config.FURNITURE_FOOTPRINTS_M[item.type]
        return "box", width, depth
    if item.shape == "circular":
        if item.radius_m is None or item.width_m is not None or item.depth_m is not None:
            raise ValueError("A circular pillar needs radius_m only.")
        return "circle", item.radius_m, item.radius_m
    if item.shape == "square":
        if item.width_m is None or item.depth_m is None or item.radius_m is not None:
            raise ValueError("A square pillar needs width_m and depth_m only.")
        if abs(item.yaw_rad) > 1e-9:
            raise ValueError("Square pillars must be axis aligned.")
        return "box", item.width_m, item.depth_m
    raise ValueError("A pillar needs shape 'circular' or 'square'.")


def _half_extents(item: WorldObject, shape: str, a: float, b: float) -> tuple[float, float]:
    if shape == "circle":
        return a, a
    cosine, sine = abs(math.cos(item.yaw_rad)), abs(math.sin(item.yaw_rad))
    return (cosine * a + sine * b) / 2, (sine * a + cosine * b) / 2


def _overlap(first: tuple[WorldObject, str, float, float], second: tuple[WorldObject, str, float, float]) -> bool:
    a, a_shape, a_x, a_y = first
    b, b_shape, b_x, b_y = second
    margin = 0.001
    if a_shape == b_shape == "circle":
        return math.hypot(a.x_m - b.x_m, a.y_m - b.y_m) < a_x + b_x + margin
    if a_shape == "circle" or b_shape == "circle":
        circle, radius, box, half_x, half_y = (
            (a, a_x, b, b_x, b_y) if a_shape == "circle" else (b, b_x, a, a_x, a_y)
        )
        closest_x = max(box.x_m - half_x, min(circle.x_m, box.x_m + half_x))
        closest_y = max(box.y_m - half_y, min(circle.y_m, box.y_m + half_y))
        return math.hypot(circle.x_m - closest_x, circle.y_m - closest_y) < radius + margin
    return abs(a.x_m - b.x_m) < a_x + b_x + margin and abs(a.y_m - b.y_m) < a_y + b_y + margin


def _pose(parent: ET.Element, values: tuple[float, ...]) -> None:
    ET.SubElement(parent, "pose").text = " ".join(f"{value:.12g}" for value in values)


def _add_object(world: ET.Element, item: WorldObject, name: str, floor_z: float, ceiling_z: float) -> float:
    if item.type != "pillar":
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "name").text = name
        ET.SubElement(include, "uri").text = f"model://{config.FURNITURE_MODELS[item.type]}"
        _pose(include, (item.x_m, item.y_m, floor_z, 0.0, 0.0, item.yaw_rad))
        return floor_z
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    height = ceiling_z - floor_z
    center_z = (floor_z + ceiling_z) / 2
    _pose(model, (item.x_m, item.y_m, center_z, 0.0, 0.0, item.yaw_rad))
    link = ET.SubElement(model, "link", {"name": "link"})
    for tag in ("collision", "visual"):
        part = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(part, "geometry")
        if item.shape == "circular":
            cylinder = ET.SubElement(geometry, "cylinder")
            ET.SubElement(cylinder, "radius").text = f"{item.radius_m:.12g}"
            ET.SubElement(cylinder, "length").text = f"{height:.12g}"
        else:
            box = ET.SubElement(geometry, "box")
            ET.SubElement(box, "size").text = f"{item.width_m:.12g} {item.depth_m:.12g} {height:.12g}"
    return center_z


def build_world_file(
    workspace_path: str,
    base_world_sdf: str,
    output_world_sdf: str,
    objects: list[WorldObject],
) -> dict[str, object]:
    """Write a validated scene while leaving the user's base room untouched."""

    workspace = Path(workspace_path).expanduser().resolve()
    base = Path(base_world_sdf).expanduser().resolve()
    output = Path(output_world_sdf).expanduser().resolve(strict=False)
    if not workspace.is_dir() or not base.is_file():
        raise ValueError("workspace_path must be a directory and base_world_sdf must be a file.")
    if base == output:
        raise ValueError("output_world_sdf must differ from the supplied room SDF.")
    try:
        output.relative_to(workspace)
    except ValueError as exc:
        raise ValueError("output_world_sdf must be inside the ROS workspace.") from exc
    try:
        tree = ET.parse(base)
    except ET.ParseError as exc:
        raise ValueError(f"Unable to parse room SDF: {exc}") from exc
    world = tree.getroot().find("world")
    if world is None or world.get("name") != "environment":
        raise ValueError("The supplied room SDF needs a world named 'environment'.")
    if len(world.findall("model")) != 1 or world.find("include") is not None:
        raise ValueError("The supplied room SDF must be empty apart from its room model.")
    x_min, x_max, y_min, y_max, z_min, z_max = _room_bounds(world)
    placed: list[tuple[WorldObject, str, float, float]] = []
    names = {"room"}
    result_objects: list[dict[str, object]] = []
    counters: dict[str, int] = {}
    for item in objects:
        counters[item.type] = counters.get(item.type, 0) + 1
        name = item.name or f"{item.type}_{counters[item.type]}"
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9_]*", name) is None or name in names:
            raise ValueError(f"Object name {name!r} must be unique and use letters, digits, or underscores.")
        names.add(name)
        shape, a, b = _footprint(item)
        half_x, half_y = _half_extents(item, shape, a, b)
        if not (x_min + half_x <= item.x_m <= x_max - half_x and y_min + half_y <= item.y_m <= y_max - half_y):
            raise ValueError(f"Object {name!r} extends outside the room.")
        candidate = (item, shape, half_x, half_y)
        for prior in placed:
            if _overlap(candidate, prior):
                other = prior[0]
                raise ValueError(f"Object {name!r} overlaps {other.name or other.type!r}.")
        placed.append(candidate)
        z_m = _add_object(world, item, name, z_min, z_max)
        result_objects.append({"name": name, "type": item.type, "x_m": item.x_m, "y_m": item.y_m, "z_m": z_m, "yaw_rad": item.yaw_rad})
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{output.stem}.", suffix=".sdf", dir=output.parent)
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        tree.write(temporary, encoding="utf-8", xml_declaration=True)
        with temporary.open("ab") as stream:
            stream.write(b"\n")
        env = os.environ.copy()
        env["SDF_PATH"] = os.pathsep.join((str(config.ENVIRONMENT_MODELS), str(base.parent), env.get("SDF_PATH", "")))
        check = subprocess.run(["gz", "sdf", "-k", str(temporary)], capture_output=True, text=True, timeout=20, env=env)
        if check.returncode:
            raise ValueError("Gazebo rejected the generated world SDF: " + (check.stderr or check.stdout).strip()[-1600:])
        os.replace(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)
    return {"world_sdf_path": str(output), "world_name": "environment", "objects": result_objects}
