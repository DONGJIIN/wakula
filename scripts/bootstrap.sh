#!/usr/bin/env bash
# 新电脑首次使用：安装 package.xml 声明的系统依赖，然后调用统一构建脚本。
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# ROS 环境脚本可能读取未定义变量，因此 source 完成后才启用 set -u。
source /opt/ros/jazzy/setup.bash
set -u
cd "$workspace_dir"
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
"${workspace_dir}/scripts/build.sh"
