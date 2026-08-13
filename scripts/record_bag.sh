#!/usr/bin/env bash
# 记录算法复现所需的原始输入、中间证据、最终约束和 TF。
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 第一个参数可指定外置硬盘目录；省略时写入工作空间 bags/。
bag_root="${1:-${workspace_dir}/bags}"
mkdir -p "$bag_root"
bag_name="wakula_$(date +%Y%m%d_%H%M%S)"

# 同时记录原始输入、标定信息、TF、感知结果和导航状态，确保问题可离线复现。
ros2 bag record --output "${bag_root}/${bag_name}" \
  /scan /odom /tf /tf_static \
  /camera/image_raw /camera/color/image_raw /camera/camera_info \
  /camera/depth/points /camera/depth/color/points \
  /terrain/features /terrain/features_stamped \
  /vision/obstacle_evidence /vision/obstacle_stamped \
  /perception/fused_obstacle /perception/obstacle_points \
  /terrain/navigation_mode /terrain/speed_limit /terrain/visual_assist_active \
  /terrain/navigation_safety \
  /traversal/guidance /traversal/phase /traversal/approach_pose \
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel_terrain_safe /cmd_vel \
  /navigation/healthy /diagnostics
