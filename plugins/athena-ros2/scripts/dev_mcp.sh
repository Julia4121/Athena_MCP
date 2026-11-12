#!/usr/bin/env bash
set -euo pipefail

plugin_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
platform_root="${ATHENA_PLATFORM_ROOT:-${HOME}/drone_platform}"
workspace="${ATHENA_WORKSPACE:-${platform_root}/.workspaces/development}"
if [ ! -x "${platform_root}/containers/development/dev_shell.sh" ]; then
  echo "Set ATHENA_PLATFORM_ROOT to the drone_platform checkout" >&2
  exit 1
fi

ATHENA_PLUGIN_ROOT="$plugin_root" WORKSPACE="$workspace" NAME=drone-development \
  ROS_DOMAIN_ID=145 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  "${platform_root}/containers/development/dev_shell.sh" --start >&2

mount_source="$(docker inspect drone-development --format '{{range .Mounts}}{{if eq .Destination "/opt/athena_ros2_plugin"}}{{.Source}}{{end}}{{end}}')"
if [ "$mount_source" != "$plugin_root" ]; then
  echo "drone-development has a stale plugin mount; stop it with dev_shell.sh --stop and retry" >&2
  exit 1
fi
workspace_source="$(docker inspect drone-development --format '{{range .Mounts}}{{if eq .Destination "/home/ws"}}{{.Source}}{{end}}{{end}}')"
if [ "$workspace_source" != "$(realpath "$workspace")" ]; then
  echo "drone-development mounts a different workspace; stop it and retry" >&2
  exit 1
fi
if ! docker exec drone-development bash -lc 'command -v gz >/dev/null && command -v MicroXRCEAgent >/dev/null && command -v uv >/dev/null && test -x /opt/PX4-Autopilot/build/px4_sitl_default/bin/px4'; then
  echo "Rebuild drone-development-jazzy:latest with the updated Dockerfile" >&2
  exit 1
fi

exec docker exec -i --user "$(id -u):$(id -g)" --workdir /opt/athena_ros2_plugin \
  -e UV_PROJECT_ENVIRONMENT=/home/ws/.athena-mcp-venv \
  -e UV_CACHE_DIR=/home/ws/.athena-uv-cache \
  drone-development uv run --project /opt/athena_ros2_plugin --locked \
  python /opt/athena_ros2_plugin/server.py
