#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$workspace_dir"
rosdep install --from-paths src slam --ignore-src --rosdistro jazzy -r -y
