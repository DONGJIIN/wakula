#!/usr/bin/env bash
# 以 /clock 重放录制数据；算法端必须使用 use_sim_time:=true 才能正确判断消息新鲜度。
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 新终端不能假定用户已经 source。系统环境提供 ros2 bag，工作空间环境提供录包中
# quadruped_interfaces 等自定义类型；缺少 install 时仍可回放只含标准消息的 bag。
source /opt/ros/jazzy/setup.bash
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi
set -u

if [[ $# -lt 1 ]]; then
  echo "用法: $0 BAG目录 [播放倍率]" >&2
  exit 2
fi

# --clock 让所有 use_sim_time:=true 的算法使用 bag 时间；暂停启动便于先拉起算法节点。
exec ros2 bag play "$1" --clock --rate "${2:-1.0}" --start-paused
