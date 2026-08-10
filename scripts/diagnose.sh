#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi

echo "ROS_DISTRO=${ROS_DISTRO:-unset} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"
ros2 doctor --report
ros2 topic list --types
echo "关键 TF（失败会返回非零）："
timeout 3 ros2 run tf2_ros tf2_echo map base_link -r 1 || true
