#!/usr/bin/env bash
# 新电脑首次使用：安装 package.xml 声明的系统依赖，然后调用统一构建脚本。
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS 环境脚本可能读取未定义变量，因此 source 完成后才启用 set -u。
source /opt/ros/jazzy/setup.bash
set -u
cd "$workspace_dir"
# ``ament_python`` 是 package.xml 中的 colcon 构建类型，不是可由
# ``ros2 pkg prefix`` 查找或由 rosdep 安装的 ROS 包。直接导入实际负责
# 该构建类型的 colcon 扩展，可以在不依赖发行版虚构包名的情况下发现缺失。
if ! python3 -c 'import colcon_ros.task.ament_python.build' >/dev/null 2>&1; then
  echo "缺少 colcon 的 ament_python 构建扩展；请安装 python3-colcon-ros。" >&2
  exit 2
fi
# 不使用 ``-r`` 或 ``--skip-keys`` 隐藏未知依赖；所有运行依赖都必须
# 在新机安装时完整解析。
rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
"${workspace_dir}/scripts/build.sh"
