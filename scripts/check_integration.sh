#!/usr/bin/env bash
# 检查未来真机是否遵守 wakula 的最小 ROS 2 对接合同。
#
# 用法：./scripts/check_integration.sh [图像话题] [点云话题]
# 本脚本只读，不启动或控制机器人；任何缺失项都会汇总并以非零状态退出。
set -o pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi
# ROS 的环境脚本会读取若干可选变量；完成 source 后再启用未定义变量检查。
set -u

camera_topic="${1:-/camera/image_raw}"
points_topic="${2:-/camera/depth/points}"
failure_count=0

check_topic() {
  local topic="$1"
  local expected_type="$2"
  local actual_type
  actual_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
  if [[ "${actual_type}" == "${expected_type}" ]]; then
    echo "[OK] ${topic} -> ${expected_type}"
  else
    echo "[缺失/类型错误] ${topic}: 期望 ${expected_type}，实际 ${actual_type:-无}"
    failure_count=$((failure_count + 1))
  fi
}

check_tf() {
  local parent="$1"
  local child="$2"
  local report_file
  report_file="$(mktemp)"
  timeout 3 ros2 run tf2_ros tf2_echo "${parent}" "${child}" \
    >"${report_file}" 2>&1 || true
  if grep -q "Translation:" "${report_file}"; then
    echo "[OK] TF ${parent} -> ${child}"
  else
    echo "[缺失] TF ${parent} -> ${child}"
    failure_count=$((failure_count + 1))
  fi
  rm -f "${report_file}"
}

echo "检查标准传感器、算法输出和真机速度接口……"
check_topic "/scan" "sensor_msgs/msg/LaserScan"
check_topic "/odom" "nav_msgs/msg/Odometry"
check_topic "${camera_topic}" "sensor_msgs/msg/Image"
check_topic "${points_topic}" "sensor_msgs/msg/PointCloud2"
check_topic "/perception/fused_obstacle" "quadruped_interfaces/msg/FusedObstacle"
check_topic "/terrain/navigation_safety" "quadruped_interfaces/msg/NavigationSafety"
check_topic "/navigation/healthy" "std_msgs/msg/Bool"
check_topic "/cmd_vel" "geometry_msgs/msg/Twist"
check_tf "map" "base_link"
check_tf "odom" "base_link"

subscriber_count="$(
  ros2 topic info /cmd_vel 2>/dev/null \
    | awk '/Subscription count:/ {print $3}' \
    | tail -n 1
)"
if [[ "${subscriber_count:-0}" =~ ^[0-9]+$ ]] \
  && ((subscriber_count > 0)); then
  echo "[OK] /cmd_vel 有 ${subscriber_count} 个订阅者"
else
  echo "[缺失] /cmd_vel 尚无真机底盘/运动控制订阅者"
  failure_count=$((failure_count + 1))
fi

if ((failure_count > 0)); then
  echo "对接检查失败：共 ${failure_count} 项；请按 connect.txt 修正 profile/remap/TF。"
  exit 1
fi
echo "对接检查通过：感知、导航与未来真机的最小合同已对齐。"
