#!/usr/bin/env bash
# 仿真键盘遥控快捷入口。必须在当前终端运行，才能直接读取 i/j/k/l 等按键。
# ROS 2 生成的 setup.bash 会有条件读取尚未定义的环境变量，因此先只启用错误/管道检查，
# 完成环境加载后再开启 nounset；否则脚本会在键盘节点启动前直接退出。
set -eo pipefail

WAKULA_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "${WAKULA_ROOT}/install/setup.bash"
set -u

# Jazzy 自带 teleop_twist_keyboard 每次按键只发布一帧 Twist；按住时由终端键盘重复产生
# 后续事件。Gazebo 专用 mux 对该候选使用 0.7 s 墙钟超时，输入停止后负责持续归零。
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  --remap cmd_vel:=/cmd_vel_teleop
