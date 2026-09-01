#!/usr/bin/env bash
# 按 Wakula 传感器 profile/显式话题记录原始输入、中间证据、最终约束和 TF。
# 本脚本只订阅并写包，不启动算法或发布运动命令。
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi
# ROS 环境脚本可能读取未定义变量，加载完成后再开启 nounset。
set -u

bag_root="${workspace_dir}/bags"
sensor_profile="ros_default"
profiles_file="${workspace_dir}/src/slam/config/sensor_profiles.yaml"
vision_params_file="${workspace_dir}/src/quadruped_perception/config/vision.yaml"
terrain_params_file="${workspace_dir}/src/quadruped_perception/config/terrain.yaml"
scan_override=""
odom_override=""
image_override=""
points_override=""
camera_info_override=""
skip_vision="false"
print_topics="false"

usage() {
  cat <<'EOF'
用法：
  ./scripts/record_bag.sh [输出根目录] [选项]

选项：
  --output-root DIR          bag 输出根目录；兼容旧版第一个位置参数
  --profile NAME            sensor_profiles.yaml 中的 profile，默认 ros_default
  --profiles-file FILE      传感器 profile YAML
  --scan TOPIC              覆盖 LaserScan 话题
  --odom TOPIC              覆盖 Odometry 话题
  --image TOPIC             覆盖 Image 话题
  --points TOPIC            覆盖 PointCloud2 话题
  --camera-info TOPIC       覆盖 CameraInfo；省略时根据 Image 名称推导
  --vision-params-file FILE  profile 未固定 Image 时，从这里读取候选列表
  --terrain-params-file FILE profile 未固定 PointCloud2 时，从这里读取候选列表
  --skip-vision             不记录 Image、CameraInfo 和 PointCloud2
  --print-topics            只打印最终话题清单，不启动 rosbag
  -h, --help                显示帮助

示例：
  ./scripts/record_bag.sh --profile oak_d
  ./scripts/record_bag.sh /media/data/wakula_bags --profile realsense_d400
  ./scripts/record_bag.sh --scan /front/scan --image /rgb/image_raw \
    --points /depth/points --camera-info /rgb/camera_info
EOF
}

# 保留旧接口：第一个非选项位置参数仍表示输出目录。
if (($# > 0)) && [[ "$1" != -* ]]; then
  bag_root="$1"
  shift
fi

while (($# > 0)); do
  case "$1" in
    --output-root) bag_root="${2:?--output-root 缺少目录}"; shift 2 ;;
    --profile) sensor_profile="${2:?--profile 缺少名称}"; shift 2 ;;
    --profiles-file) profiles_file="${2:?--profiles-file 缺少文件}"; shift 2 ;;
    --scan) scan_override="${2:?--scan 缺少话题}"; shift 2 ;;
    --odom) odom_override="${2:?--odom 缺少话题}"; shift 2 ;;
    --image) image_override="${2:?--image 缺少话题}"; shift 2 ;;
    --points) points_override="${2:?--points 缺少话题}"; shift 2 ;;
    --camera-info) camera_info_override="${2:?--camera-info 缺少话题}"; shift 2 ;;
    --vision-params-file)
      vision_params_file="${2:?--vision-params-file 缺少文件}"; shift 2 ;;
    --terrain-params-file)
      terrain_params_file="${2:?--terrain-params-file 缺少文件}"; shift 2 ;;
    --skip-vision) skip_vision="true"; shift ;;
    --print-topics) print_topics="true"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1" >&2; usage >&2; exit 2 ;;
  esac
done

# 解析逻辑复用 slam.sensor_profiles 的校验；profile 中视觉话题为空时，再读取算法自身
# YAML 的候选列表，避免在录包脚本中维护第二套易失配的相机/点云默认值。
resolver_python_path="${workspace_dir}/src/slam${PYTHONPATH:+:${PYTHONPATH}}"
if ! resolved_topics="$(
  PYTHONPATH="${resolver_python_path}" python3 - \
    "${profiles_file}" "${sensor_profile}" \
    "${scan_override}" "${odom_override}" "${image_override}" "${points_override}" \
    "${camera_info_override}" "${vision_params_file}" "${terrain_params_file}" \
    "${skip_vision}" <<'PY'
from pathlib import Path
import sys

import yaml

from slam.sensor_profiles import load_sensor_profiles, resolve_sensor_topics


(
    profiles_path,
    profile_name,
    scan_override,
    odom_override,
    image_override,
    points_override,
    camera_info_override,
    vision_params_path,
    terrain_params_path,
    skip_vision_text,
) = sys.argv[1:]


def parameters(path, node_name):
    document = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    node = document.get(node_name, {})
    values = node.get("ros__parameters", {}) if isinstance(node, dict) else {}
    if not isinstance(values, dict):
        raise ValueError(f"{path}: {node_name}.ros__parameters must be a map")
    return values


def validate_topics(values, label):
    topics = []
    for raw_topic in values:
        topic = str(raw_topic).strip()
        if (
            not topic.startswith("/")
            or topic.endswith("/")
            or "//" in topic
            or any(character.isspace() for character in topic)
        ):
            raise ValueError(f"{label} contains an invalid absolute topic: {raw_topic!r}")
        if topic not in topics:
            topics.append(topic)
    return topics


def camera_info_for(image_topic):
    parent, separator, leaf = image_topic.rpartition("/")
    if not separator or leaf not in {"image", "image_raw", "image_rect", "image_rect_raw"}:
        return ""
    return f"{parent}/camera_info" if parent else "/camera_info"


profiles = load_sensor_profiles(profiles_path)
resolved = resolve_sensor_topics(
    profiles,
    profile_name,
    {
        "scan_topic": scan_override,
        "odom_topic": odom_override,
        "camera_topic": image_override,
        "point_cloud_topic": points_override,
    },
)
print("scan\t" + resolved["scan_topic"])
print("odom\t" + resolved["odom_topic"])

if skip_vision_text != "true":
    if resolved["camera_topic"]:
        image_topics = [resolved["camera_topic"]]
    else:
        vision = parameters(vision_params_path, "vision_obstacle_detector")
        image_topics = validate_topics(vision.get("image_topic_candidates", []), "image candidates")

    if resolved["point_cloud_topic"]:
        point_topics = [resolved["point_cloud_topic"]]
    else:
        terrain = parameters(terrain_params_path, "terrain_analyzer")
        point_topics = validate_topics(
            terrain.get("input_topic_candidates", []), "point-cloud candidates"
        )

    for topic in image_topics:
        print("image\t" + topic)
    for topic in point_topics:
        print("points\t" + topic)

    if camera_info_override:
        info_topics = validate_topics([camera_info_override], "camera info")
    else:
        info_topics = validate_topics(
            [topic for topic in map(camera_info_for, image_topics) if topic],
            "derived camera info",
        )
    for topic in info_topics:
        print("camera_info\t" + topic)
PY
)"; then
  echo "无法解析录包传感器话题；请检查 profile 和参数文件。" >&2
  exit 2
fi

declare -a topics=()
declare -A seen_topics=()
add_topic() {
  local topic="$1"
  if [[ -n "${topic}" && -z "${seen_topics[${topic}]+present}" ]]; then
    topics+=("${topic}")
    seen_topics["${topic}"]=1
  fi
}

while IFS=$'\t' read -r _kind topic; do
  add_topic "${topic}"
done <<<"${resolved_topics}"

# TF 与 /clock 让同一 bag 可重建坐标和仿真时间；不存在的发布者不会阻止 rosbag 等待。
for topic in /tf /tf_static /clock \
  /terrain/features /terrain/features_stamped \
  /vision/obstacle_evidence /vision/obstacle_stamped \
  /perception/fused_obstacle /perception/obstacle_points \
  /terrain/navigation_mode /terrain/speed_limit /terrain/visual_assist_active \
  /terrain/navigation_safety \
  /traversal/guidance /traversal/phase /traversal/approach_pose \
  /cmd_vel_nav /cmd_vel_smoothed /cmd_vel \
  /navigation/healthy /diagnostics; do
  add_topic "${topic}"
done

echo "sensor_profile=${sensor_profile}"
printf 'record_topic=%s\n' "${topics[@]}"
if [[ "${print_topics}" == "true" ]]; then
  exit 0
fi

mkdir -p "${bag_root}"
bag_name="wakula_$(date +%Y%m%d_%H%M%S)"
exec ros2 bag record --output "${bag_root}/${bag_name}" "${topics[@]}"
