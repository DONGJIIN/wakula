#!/usr/bin/env bash
# 检查任意机器人是否满足 wakula SLAM/Nav2/OpenCV 的最小 ROS 2 对接合同。
#
# 本脚本严格只读：不启动算法、不发送速度，也不改变任何节点。它提供两种检查层级：
#   --inputs-only  先只检查硬件驱动、消息内容和传感器 TF，适合移植第一步；
#   默认           算法启动后再检查 map TF、算法输出及 /cmd_vel 消费者。
#
# 旧版的两个位置参数仍兼容：check_integration.sh IMAGE_TOPIC POINTS_TOPIC。
# 新接入建议使用具名参数，避免队员记错参数顺序。运行 --help 查看完整示例。
set -o pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi
# ROS 环境脚本会读取一些未必存在的可选变量，因此应在 source 完成后才启用 -u。
set -u

scan_topic="/scan"
odom_topic="/odom"
camera_topic="/camera/image_raw"
points_topic="/camera/depth/points"
cmd_vel_topic="/cmd_vel"
map_frame="map"
odom_frame="odom"
base_frame="base_link"
inputs_only=false
skip_vision=false
require_traverse_server=false
failure_count=0
warning_count=0

usage() {
  cat <<'EOF'
用法：
  ./scripts/check_integration.sh [IMAGE_TOPIC POINTS_TOPIC]
  ./scripts/check_integration.sh [选项]

选项：
  --inputs-only              仅检查传感器、里程计和传感器 TF
  --skip-vision              不要求相机图像和深度/雷达点云
  --require-traverse-server  要求真机提供 /traverse_obstacle Action 服务端
  --scan TOPIC               LaserScan 话题，默认 /scan
  --odom TOPIC               Odometry 话题，默认 /odom
  --image TOPIC              Image 话题，默认 /camera/image_raw
  --points TOPIC             PointCloud2 话题，默认 /camera/depth/points
  --cmd-vel TOPIC            速度输出话题，默认 /cmd_vel
  --map-frame FRAME          地图坐标系，默认 map
  --odom-frame FRAME         里程计坐标系，默认 odom
  --base-frame FRAME         机体坐标系，默认 base_link
  -h, --help                 显示帮助

推荐迁移顺序：
  ./scripts/check_integration.sh --inputs-only --image /实际图像 --points /实际点云
  ros2 launch slam slam.launch.py robot_model:=false camera_topic:=/实际图像 \
    point_cloud_topic:=/实际点云
  ./scripts/check_integration.sh --image /实际图像 --points /实际点云
EOF
}

# 向后兼容旧文档和已有使用习惯；具名选项仍是新代码推荐方式。
if (($# > 0)) && [[ "$1" != -* ]]; then
  camera_topic="$1"
  shift
  if (($# > 0)) && [[ "$1" != -* ]]; then
    points_topic="$1"
    shift
  fi
fi

while (($# > 0)); do
  case "$1" in
    --inputs-only) inputs_only=true; shift ;;
    --skip-vision) skip_vision=true; shift ;;
    --require-traverse-server) require_traverse_server=true; shift ;;
    --scan) scan_topic="${2:?--scan 缺少话题名}"; shift 2 ;;
    --odom) odom_topic="${2:?--odom 缺少话题名}"; shift 2 ;;
    --image) camera_topic="${2:?--image 缺少话题名}"; shift 2 ;;
    --points) points_topic="${2:?--points 缺少话题名}"; shift 2 ;;
    --cmd-vel) cmd_vel_topic="${2:?--cmd-vel 缺少话题名}"; shift 2 ;;
    --map-frame) map_frame="${2:?--map-frame 缺少坐标系}"; shift 2 ;;
    --odom-frame) odom_frame="${2:?--odom-frame 缺少坐标系}"; shift 2 ;;
    --base-frame) base_frame="${2:?--base-frame 缺少坐标系}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1"; usage; exit 2 ;;
  esac
done

# 每个进程只有一个临时目录，退出（包括 Ctrl-C）时统一回收采样结果。
report_dir="$(mktemp -d)"
trap 'rm -rf -- "${report_dir}"' EXIT

fail() {
  echo "[失败] $*"
  failure_count=$((failure_count + 1))
}

warn() {
  echo "[警告] $*"
  warning_count=$((warning_count + 1))
}

topic_type() {
  local topic="$1"
  local discovered_type=""
  local attempt
  # DDS participant 刚启动时，命令行进程可能比 discovery 更早查询到空结果。这里允许最多
  # 5 秒收敛，但一旦发现类型立即返回；既避免仿真/真机冷启动误报，也不会无限掩盖断流。
  for attempt in {1..10}; do
    discovered_type="$(ros2 topic type "${topic}" 2>/dev/null || true)"
    if [[ -n "${discovered_type}" ]]; then
      printf '%s' "${discovered_type}"
      return
    fi
    sleep 0.5
  done
  printf '%s' ""
}

check_topic_type() {
  local topic="$1"
  local expected_type="$2"
  local actual_type
  actual_type="$(topic_type "${topic}")"
  if [[ "${actual_type}" == "${expected_type}" ]]; then
    echo "[通过] ${topic} -> ${expected_type}"
    return 0
  fi
  fail "${topic} 期望 ${expected_type}，实际 ${actual_type:-无发布者}"
  return 1
}

# 类型正确仍不代表数据可用：驱动可能只注册了发布者却没有持续发送。这里用 best_effort
# 订阅兼容常见 Sensor Data QoS，并以 5 秒为上限抓取一帧，防止检查脚本无限卡住。
capture_message() {
  local topic="$1"
  local output_file="$2"
  if timeout 5 ros2 topic echo "${topic}" --once \
      --qos-reliability best_effort >"${output_file}" 2>/dev/null; then
    return 0
  fi
  fail "${topic} 在 5 秒内没有收到消息（检查驱动、QoS、时间源和线路）"
  return 1
}

first_yaml_value() {
  local key="$1"
  local file="$2"
  awk -v wanted="${key}:" '$1 == wanted {print $2; exit}' "${file}" \
    | tr -d "'\""
}

# 检查带 std_msgs/Header 的传感器消息，并通过返回变量暴露实际 frame_id，供后续 TF
# 连通性检查使用。ROS 2 echo 的 YAML 缩进可能随发行版变化，因此只匹配键，不依赖缩进。
check_header_topic() {
  local topic="$1"
  local expected_type="$2"
  local label="$3"
  local result_variable="$4"
  local output_file="${report_dir}/${label}.yaml"
  local frame_id
  if ! check_topic_type "${topic}" "${expected_type}"; then
    printf -v "${result_variable}" '%s' ""
    return
  fi
  if ! capture_message "${topic}" "${output_file}"; then
    printf -v "${result_variable}" '%s' ""
    return
  fi
  frame_id="$(first_yaml_value frame_id "${output_file}")"
  if [[ -z "${frame_id}" ]]; then
    fail "${topic} 的 header.frame_id 为空，算法无法把数据变换到 ${base_frame}"
    printf -v "${result_variable}" '%s' ""
    return
  fi
  echo "[通过] ${topic} 有实时数据，frame_id=${frame_id}"
  printf -v "${result_variable}" '%s' "${frame_id}"
}

check_tf() {
  local parent="$1"
  local child="$2"
  local label="$3"
  local report_file="${report_dir}/tf_${label}.txt"
  if [[ -z "${child}" ]]; then
    return
  fi
  # tf2_echo 每次都是新的 DDS participant，静态 TF 需要等待 discovery 后才能收到
  # transient-local 数据。8 秒上限覆盖普通 Wi-Fi/低功耗板冷启动，同时仍保证脚本有界退出。
  timeout 8 ros2 run tf2_ros tf2_echo "${parent}" "${child}" \
    >"${report_file}" 2>&1 || true
  if grep -q "Translation:" "${report_file}"; then
    echo "[通过] TF ${parent} -> ${child}"
  else
    fail "缺少 TF ${parent} -> ${child}"
  fi
}

echo "=== wakula 可移植算法对接检查 ==="
if ${inputs_only}; then
  echo "模式：硬件输入（算法可暂未启动）"
else
  echo "模式：完整算法链路"
fi

scan_frame=""
odom_header_frame=""
camera_frame=""
points_frame=""
check_header_topic "${scan_topic}" "sensor_msgs/msg/LaserScan" scan scan_frame
check_header_topic "${odom_topic}" "nav_msgs/msg/Odometry" odom odom_header_frame

# Odometry 除了 header.frame_id=odom，还必须声明 child_frame_id=base_link；这两个字段
# 写错会造成 SLAM 建图滞后、旋转丢图或 Nav2 认为机器人不动。
odom_sample="${report_dir}/odom.yaml"
if [[ -s "${odom_sample}" ]]; then
  odom_child_frame="$(first_yaml_value child_frame_id "${odom_sample}")"
  if [[ "${odom_header_frame}" != "${odom_frame}" ]]; then
    fail "${odom_topic} 的 header.frame_id=${odom_header_frame:-空}，期望 ${odom_frame}"
  fi
  if [[ "${odom_child_frame}" != "${base_frame}" ]]; then
    fail "${odom_topic} 的 child_frame_id=${odom_child_frame:-空}，期望 ${base_frame}"
  fi
fi

if ! ${skip_vision}; then
  check_header_topic "${camera_topic}" "sensor_msgs/msg/Image" image camera_frame
  check_header_topic "${points_topic}" "sensor_msgs/msg/PointCloud2" points points_frame
else
  warn "已跳过视觉/点云检查；SLAM 可运行，但 OpenCV 与地形分类不会产生有效结果"
fi

check_tf "${odom_frame}" "${base_frame}" odom_base
check_tf "${base_frame}" "${scan_frame}" base_scan
if ! ${skip_vision}; then
  check_tf "${base_frame}" "${camera_frame}" base_camera
  check_tf "${base_frame}" "${points_frame}" base_points
fi

if ! ${inputs_only}; then
  check_tf "${map_frame}" "${base_frame}" map_base
  check_topic_type "/perception/fused_obstacle" "quadruped_interfaces/msg/FusedObstacle"
  check_topic_type "/terrain/navigation_safety" "quadruped_interfaces/msg/NavigationSafety"
  check_topic_type "/traversal/guidance" "quadruped_interfaces/msg/TraversalGuidance"
  check_topic_type "/navigation/healthy" "std_msgs/msg/Bool"
  check_topic_type "${cmd_vel_topic}" "geometry_msgs/msg/Twist"

  subscriber_count="$(
    ros2 topic info "${cmd_vel_topic}" 2>/dev/null \
      | awk '/Subscription count:/ {print $3}' \
      | tail -n 1
  )"
  if [[ "${subscriber_count:-0}" =~ ^[0-9]+$ ]] && ((subscriber_count > 0)); then
    echo "[通过] ${cmd_vel_topic} 有 ${subscriber_count} 个运动控制订阅者"
  else
    fail "${cmd_vel_topic} 没有运动控制订阅者；算法算出的速度无法到达真机"
  fi

  action_type="$(ros2 action type /traverse_obstacle 2>/dev/null || true)"
  if [[ "${action_type}" == "quadruped_interfaces/action/TraverseObstacle" ]]; then
    echo "[通过] /traverse_obstacle -> ${action_type}"
  elif ${require_traverse_server}; then
    fail "/traverse_obstacle Action 服务端缺失或类型错误"
  else
    warn "/traverse_obstacle 服务端尚未接入；SLAM/Nav2 可用，真实越障动作仍由运动组实现"
  fi
fi

echo "=== 检查完成：失败 ${failure_count}，警告 ${warning_count} ==="
if ((failure_count > 0)); then
  echo "对接未通过：请按 connect.txt 修正 profile/remap、消息帧或 TF。"
  exit 1
fi
echo "对接通过：当前机器满足所选检查层级的算法合同。"
