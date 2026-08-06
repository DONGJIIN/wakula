#!/usr/bin/env bash
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
cd "$workspace_dir"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
