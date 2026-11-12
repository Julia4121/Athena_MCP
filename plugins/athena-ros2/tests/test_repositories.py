"""Repository placement checks for bringup and new feature packages."""

from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from repositories import prepare_bringup, prepare_feature, repository_layout


def _repo(path: Path, *packages: str) -> None:
    path.mkdir(parents=True)
    subprocess.run(["git", "init", "-q", "-b", "main", str(path)], check=True)
    for package in packages:
        source = path / package
        source.mkdir()
        (source / "package.xml").write_text(f"<package><name>{package}</name></package>\n")


class RepositoryPlacementTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.workspace = Path(self.temporary.name)
        (self.workspace / "src").mkdir()
        self.template = self.workspace / "template"
        self.template.mkdir()
        (self.template / "package.xml").write_text("<package><name>sim_bringup</name></package>\n")

    def test_reuses_existing_athena_repository_and_seeds_sim_bringup(self) -> None:
        root = self.workspace / "src/athena_01"
        _repo(root, "atlas_bringup")
        result = prepare_bringup(self.workspace, "Julia4121", self.template)
        self.assertFalse(result["created_repository"])
        self.assertTrue(result["atlas_bringup_present"])
        self.assertTrue((root / "sim_bringup/package.xml").is_file())

    def test_creates_athena_repository_when_missing(self) -> None:
        with patch("repositories._token_for_owner", return_value="test-token"), patch(
            "repositories._create_private_repository", return_value="git@github.com:Julia4121/athena_01.git"
        ) as create:
            result = prepare_bringup(self.workspace, "Julia4121", self.template)
        create.assert_called_once_with("athena_01", "Julia4121", "test-token")
        self.assertTrue(result["created_repository"])
        self.assertFalse(result["atlas_bringup_present"])
        self.assertTrue((self.workspace / "src/athena_01/sim_bringup/package.xml").is_file())

    def test_adds_feature_to_single_repository_workspace(self) -> None:
        root = self.workspace / "src/drone_workspace"
        _repo(root, "camera", "flight_controller")
        result = prepare_feature(self.workspace, "new_sensor", "Julia4121")
        self.assertEqual(repository_layout(self.workspace)[0], "monorepo")
        self.assertEqual(result["repository"], str(root))
        self.assertEqual(result["package_path"], str(root / "new_sensor"))
        self.assertFalse(result["created_repository"])

    def test_adds_feature_under_src_when_workspace_itself_is_one_repository(self) -> None:
        subprocess.run(["git", "init", "-q", "-b", "main", str(self.workspace)], check=True)
        package = self.workspace / "src/camera"
        package.mkdir()
        (package / "package.xml").write_text("<package><name>camera</name></package>\n")
        result = prepare_feature(self.workspace, "new_sensor", "Julia4121")
        self.assertEqual(repository_layout(self.workspace), ("monorepo", self.workspace))
        self.assertEqual(result["package_path"], str(self.workspace / "src/new_sensor"))
        self.assertFalse(result["created_repository"])

    def test_creates_named_repo_in_separate_repository_workspace(self) -> None:
        for name in ("drone_bringup", "px4_ros_bridge", "offboard_trajectory"):
            _repo(self.workspace / "src" / name, name)
        with patch("repositories._token_for_owner", return_value="test-token"), patch(
            "repositories._create_private_repository", return_value="git@github.com:Julia4121/new_sensor.git"
        ) as create:
            result = prepare_feature(self.workspace, "new_sensor", "Julia4121")
        create.assert_called_once_with("new_sensor", "Julia4121", "test-token")
        self.assertEqual(result["repository"], str(self.workspace / "src/new_sensor"))
        self.assertEqual(result["package_path"], str(self.workspace / "src/new_sensor"))
        self.assertTrue(result["created_repository"])

    def test_mixed_layout_requires_a_choice(self) -> None:
        _repo(self.workspace / "src/multi", "camera", "controller")
        _repo(self.workspace / "src/sensor", "sensor")
        with self.assertRaisesRegex(ValueError, "Mixed ROS repository layout"):
            repository_layout(self.workspace)


if __name__ == "__main__":
    unittest.main()
