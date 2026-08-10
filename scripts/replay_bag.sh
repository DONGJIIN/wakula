#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "用法: $0 BAG目录 [播放倍率]" >&2
  exit 2
fi

# --clock 让所有 use_sim_time:=true 的算法使用 bag 时间；暂停启动便于先拉起算法节点。
ros2 bag play "$1" --clock --rate "${2:-1.0}" --start-paused
