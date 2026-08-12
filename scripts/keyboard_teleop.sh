#!/usr/bin/env bash
# 仿真键盘遥控快捷入口。必须在当前终端运行，才能直接读取 i/j/k/l 等按键。
# ROS 2 生成的 setup.bash 会有条件读取尚未定义的环境变量，因此先只启用错误/管道检查，
# 完成环境加载后再开启 nounset；否则脚本会在键盘节点启动前直接退出。
set -eo pipefail

WAKULA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${WAKULA_ROOT}/install/setup.bash"
set -u

# 独立手动话题由 Gazebo 专用 mux 赋予短时最高优先级，避免与 Collision Monitor 的
# /cmd_vel 零速度竞争。repeat_rate 保证按住键时连续运动，key_timeout 负责松键停车。
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  --remap cmd_vel:=/cmd_vel_teleop \
  --param repeat_rate:=20.0 \
  --param key_timeout:=0.6
