"""Behavioral checks for scene placement and input preservation."""

from __future__ import annotations

import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

from simulation.world import WorldObject, build_world_file


def _empty_room(path: Path, width: float = 34, depth: float = 15.5, height: float = 10.8) -> None:
    sdf = ET.Element("sdf", {"version": "1.9"})
    world = ET.SubElement(sdf, "world", {"name": "environment"})
    model = ET.SubElement(world, "model", {"name": "room"})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "walls"})
    geometry = {
        "floor": ((0, 0, -0.1), (width, depth, 0.2)),
        "ceiling": ((0, 0, height + 0.1), (width, depth, 0.2)),
        "wall_x_min": ((-width / 2 - 0.1, 0, height / 2), (0.2, depth, height)),
        "wall_x_max": ((width / 2 + 0.1, 0, height / 2), (0.2, depth, height)),
        "wall_y_min": ((0, -depth / 2 - 0.1, height / 2), (width, 0.2, height)),
        "wall_y_max": ((0, depth / 2 + 0.1, height / 2), (width, 0.2, height)),
    }
    for name, (pose, size) in geometry.items():
        collision = ET.SubElement(link, "collision", {"name": name + "_collision"})
        ET.SubElement(collision, "pose").text = " ".join(map(str, (*pose, 0, 0, 0)))
        box = ET.SubElement(ET.SubElement(collision, "geometry"), "box")
        ET.SubElement(box, "size").text = " ".join(map(str, size))
    ET.ElementTree(sdf).write(path, encoding="utf-8", xml_declaration=True)


class BuildWorldTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.workspace = Path(self.temporary.name)
        self.base = self.workspace / "room.sdf"
        self.output = self.workspace / "tests" / "scene.sdf"
        _empty_room(self.base)
        self.original = self.base.read_bytes()

    def test_places_all_supported_objects_without_changing_room(self) -> None:
        objects = [
            WorldObject(type="pillar", shape="circular", radius_m=0.3, x_m=-12, y_m=0),
            WorldObject(type="table", x_m=-6, y_m=0),
            WorldObject(type="chair", x_m=0, y_m=0),
            WorldObject(type="cardboard_box", x_m=6, y_m=0),
        ]
        result = build_world_file(str(self.workspace), str(self.base), str(self.output), objects)
        self.assertEqual(self.base.read_bytes(), self.original)
        self.assertEqual(result["world_sdf_path"], str(self.output))
        self.assertEqual(len(result["objects"]), 4)
        self.assertEqual([item["z_m"] for item in result["objects"]], [5.4, 0.0, 0.0, 0.0])
        world = ET.parse(self.output).getroot().find("world")
        self.assertEqual(len(world.findall("model")), 2)
        self.assertEqual(len(world.findall("include")), 3)

    def test_rejects_overlaps_and_outside_placements(self) -> None:
        cases = [
            [WorldObject(type="chair", x_m=0, y_m=0), WorldObject(type="cardboard_box", x_m=0, y_m=0)],
            [WorldObject(type="table", x_m=17, y_m=0)],
            [WorldObject(type="chair", x_m=0, y_m=0, name="same"), WorldObject(type="chair", x_m=4, y_m=0, name="same")],
        ]
        for objects in cases:
            with self.subTest(objects=objects), self.assertRaises(ValueError):
                build_world_file(str(self.workspace), str(self.base), str(self.output), objects)
            self.assertFalse(self.output.exists())
            self.assertEqual(self.base.read_bytes(), self.original)

    def test_basement_room_limits(self) -> None:
        _empty_room(self.base, width=30, depth=2.5, height=3.5)
        result = build_world_file(
            str(self.workspace), str(self.base), str(self.output),
            [WorldObject(type="table", x_m=0, y_m=0)],
        )
        self.assertEqual(result["objects"][0]["z_m"], 0.0)
        valid_output = self.output.read_bytes()
        with self.assertRaises(ValueError):
            build_world_file(
                str(self.workspace), str(self.base), str(self.output),
                [WorldObject(type="table", x_m=0, y_m=1.0)],
            )
        self.assertEqual(self.output.read_bytes(), valid_output)


if __name__ == "__main__":
    unittest.main()
