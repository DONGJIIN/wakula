#!/usr/bin/env bash
# 增量编译整个工作空间；额外参数原样传给 colcon，例如 --packages-select slam。
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
set -u
cd "$workspace_dir"
# symlink-install 让 Python/launch/config 修改无需每次复制安装；RelWithDebInfo 兼顾性能和回溯。
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo "$@"
