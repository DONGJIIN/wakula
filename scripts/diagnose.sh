#!/usr/bin/env bash
# 只读诊断 ROS 环境、话题类型和关键定位 TF，不启动节点或发送速度。
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi

echo "ROS_DISTRO=${ROS_DISTRO:-unset} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"
# 先输出系统级 ROS 报告，再列出现有通信图，便于区分环境问题和节点问题。
ros2 doctor --report
ros2 topic list --types
echo "关键 TF（失败会返回非零）："
timeout 3 ros2 run tf2_ros tf2_echo map base_link -r 1 || true
