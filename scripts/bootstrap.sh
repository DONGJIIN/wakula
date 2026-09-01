#!/usr/bin/env bash
# 新电脑首次使用：安装 package.xml 声明的系统依赖，然后调用统一构建脚本。
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS 环境脚本可能读取未定义变量，因此 source 完成后才启用 set -u。
source /opt/ros/jazzy/setup.bash
set -u
cd "$workspace_dir"
# ament_python 是 ROS 构建类型而不是 Ubuntu rosdep key；Noble 的 rosdep 数据库会对它
# 报“无定义”，即使 /opt/ros/jazzy 中已经正确安装。先显式验证 ROS 包，再只跳过这一项。
# 不使用 rosdep 的 -r：除该已解释例外外，任何未知依赖都必须让新机安装立即失败。
if ! ros2 pkg prefix ament_python >/dev/null 2>&1; then
  echo "缺少 ament_python；请先安装 ros-jazzy-ament-python。" >&2
  exit 2
fi
rosdep install --from-paths src --ignore-src --rosdistro jazzy \
  --skip-keys ament_python -y
"${workspace_dir}/scripts/build.sh"
