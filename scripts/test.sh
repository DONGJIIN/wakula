#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$workspace_dir"
colcon test --event-handlers console_direct+ "$@"
colcon test-result --verbose
