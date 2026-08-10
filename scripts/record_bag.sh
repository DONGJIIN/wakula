#!/usr/bin/env bash
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
bag_root="${1:-${workspace_dir}/bags}"
mkdir -p "$bag_root"
bag_name="wakula_$(date +%Y%m%d_%H%M%S)"

# 同时记录原始输入、标定信息、TF、强类型结果和控制/安全状态，确保问题可离线复现。
ros2 bag record --output "${bag_root}/${bag_name}" \
  /scan /odom /tf /tf_static \
  /camera/image_raw /camera/color/image_raw /camera/camera_info \
  /camera/depth/points /camera/depth/color/points \
  /terrain/features /terrain/features_stamped \
  /vision/obstacle_evidence /vision/obstacle_stamped \
  /perception/fused_obstacle /perception/obstacle_points \
  /crossing/mode /crossing/action /crossing/execution_command \
  /crossing/execution_status /crossing/last_result \
  /cmd_vel_nav /cmd_vel /joint_states /imu/data /battery_state \
  /hardware/status /safety/stop /safety/state /navigation/healthy /diagnostics
