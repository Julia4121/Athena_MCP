"""Choose a ROS source repository and prepare Athena bringup sources."""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import urllib.error
import urllib.request
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path

ATHENA_NAME = re.compile(r"athena_(\d+)")
PACKAGE_NAME = re.compile(r"[a-z][a-z0-9_]*")
PLUGIN_ROOT = Path(__file__).resolve().parent


def git_root(path: Path) -> Path | None:
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "--show-toplevel"],
        capture_output=True, text=True, check=False,
    )
    return Path(result.stdout.strip()).resolve() if result.returncode == 0 else None


def _source_packages(workspace: Path) -> list[tuple[Path, Path]]:
    packages: list[tuple[Path, Path]] = []
    for manifest in (workspace / "src").rglob("package.xml"):
        relative = manifest.relative_to(workspace / "src")
        if any(part in {"build", "install", "log", "vendor", ".git"} for part in relative.parts):
            continue
        if any(ATHENA_NAME.fullmatch(part) for part in relative.parts):
            continue
        try:
            name = ET.parse(manifest).getroot().findtext("name")
        except ET.ParseError:
            continue
        if not name:
            continue
        root = git_root(manifest.parent)
        if root is not None and root.is_relative_to(workspace.resolve()):
            packages.append((manifest.parent, root))
    return packages


def repository_layout(workspace: Path) -> tuple[str, Path | None]:
    """Return monorepo or separate; leave an unclear mixed layout to the operator."""

    packages = _source_packages(workspace)
    roots = Counter(root for _, root in packages)
    if not packages:
        raise ValueError("No existing first-party ROS packages establish a repository layout")
    if len(roots) == 1 and (len(packages) > 1 or workspace.resolve() in roots):
        return "monorepo", next(iter(roots))
    individually_owned = sum(count for count in roots.values() if count == 1)
    if individually_owned > len(packages) / 2:
        return "separate", None
    raise ValueError("Mixed ROS repository layout; choose the package owner before creating a repository")


def _github_request(method: str, path: str, token: str, payload: dict | None = None) -> dict | list:
    request = urllib.request.Request(
        "https://api.github.com" + path,
        data=json.dumps(payload).encode() if payload is not None else None,
        method=method,
        headers={
            "Authorization": f"Bearer {token}",
            "Accept": "application/vnd.github+json",
            "X-GitHub-Api-Version": "2022-11-28",
            "Content-Type": "application/json",
        },
    )
    with urllib.request.urlopen(request, timeout=20) as response:
        return json.load(response)


def _token_for_owner(owner: str) -> str:
    token = os.environ.get("GITHUB_TOKEN", "")
    if not token:
        raise ValueError("GITHUB_TOKEN is required to create a private ROS repository")
    identity = _github_request("GET", "/user", token)
    if not isinstance(identity, dict) or str(identity.get("login", "")).casefold() != owner.casefold():
        raise ValueError(f"GITHUB_TOKEN must belong to {owner}")
    return token


def _create_private_repository(name: str, owner: str, token: str) -> str:
    response = _github_request(
        "POST", "/user/repos", token,
        {"name": name, "private": True, "auto_init": False},
    )
    if not isinstance(response, dict) or not isinstance(response.get("ssh_url"), str):
        raise RuntimeError("GitHub did not return a repository SSH URL")
    return response["ssh_url"]


def _init_checkout(path: Path, remote_url: str) -> None:
    path.mkdir(parents=True, exist_ok=False)
    for command in (
        ["git", "init", "-b", "main", str(path)],
        ["git", "-C", str(path), "remote", "add", "origin", remote_url],
    ):
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        if result.returncode:
            raise RuntimeError(f"Created {remote_url}, but local Git setup failed: {result.stderr.strip()}")


def prepare_bringup(workspace: Path, owner: str, template: Path) -> dict[str, str | bool]:
    src = workspace / "src"
    existing = sorted(
        path for path in src.iterdir()
        if path.is_dir() and ATHENA_NAME.fullmatch(path.name) and git_root(path) == path.resolve()
    ) if src.is_dir() else []
    if len(existing) > 1:
        raise ValueError("Multiple athena_NN checkouts exist; select the bringup repository explicitly")
    created = not existing
    if existing:
        root = existing[0]
    else:
        token = _token_for_owner(owner)
        numbers = [int(match.group(1)) for path in src.iterdir() if (match := ATHENA_NAME.fullmatch(path.name))]
        number = max(numbers, default=0) + 1
        for _ in range(20):
            name = f"athena_{number:02d}"
            try:
                url = _create_private_repository(name, owner, token)
                break
            except urllib.error.HTTPError as exc:
                if exc.code != 422:
                    raise
                number += 1
        else:
            raise RuntimeError("Could not allocate an unused athena_NN repository")
        root = src / name
        _init_checkout(root, url)
    other_packages = [
        path.parent.name for path in root.rglob("package.xml")
        if path.parent.name not in {"sim_bringup", "atlas_bringup"}
    ]
    if other_packages:
        raise ValueError(f"{root} contains non-bringup ROS packages: {sorted(other_packages)}")
    destination = root / "sim_bringup"
    if not destination.exists():
        if not (template / "package.xml").is_file():
            raise ValueError(f"sim_bringup template is missing: {template}")
        shutil.copytree(template, destination, ignore=shutil.ignore_patterns("__pycache__", "*.pyc"))
    return {
        "layout": "bringup",
        "repository": str(root),
        "created_repository": created,
        "sim_bringup": str(destination),
        "atlas_bringup_present": (root / "atlas_bringup/package.xml").is_file(),
    }


def prepare_feature(workspace: Path, package_name: str, owner: str) -> dict[str, str | bool]:
    if not PACKAGE_NAME.fullmatch(package_name):
        raise ValueError("package_name must be a valid lowercase ROS package name")
    if any(
        ET.parse(path).getroot().findtext("name") == package_name
        for path in (workspace / "src").rglob("package.xml")
    ):
        raise ValueError(f"ROS package {package_name} already exists; edit its owning repository")
    layout, root = repository_layout(workspace)
    if layout == "monorepo":
        assert root is not None
        package_path = (workspace / "src" / package_name) if root == workspace.resolve() else (root / package_name)
        return {
            "layout": layout,
            "repository": str(root),
            "package_path": str(package_path),
            "created_repository": False,
        }
    root = workspace / "src" / package_name
    if root.exists():
        raise ValueError(f"Checkout path already exists: {root}")
    token = _token_for_owner(owner)
    url = _create_private_repository(package_name, owner, token)
    _init_checkout(root, url)
    return {
        "layout": layout,
        "repository": str(root),
        "package_path": str(root),
        "created_repository": True,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=("bringup", "feature"))
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--package")
    parser.add_argument("--owner", default=os.environ.get("ATHENA_GITHUB_OWNER", "Julia4121"))
    parser.add_argument("--sim-template", type=Path, default=PLUGIN_ROOT / "scaffold/sim_bringup")
    args = parser.parse_args()
    workspace = args.workspace.expanduser().resolve()
    if not (workspace / "src").is_dir():
        parser.error(f"ROS workspace src directory is missing: {workspace / 'src'}")
    if args.kind == "feature" and not args.package:
        parser.error("--package is required for a feature repository")
    result = (
        prepare_bringup(workspace, args.owner, args.sim_template)
        if args.kind == "bringup"
        else prepare_feature(workspace, args.package, args.owner)
    )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
