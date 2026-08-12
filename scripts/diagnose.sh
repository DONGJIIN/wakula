#!/usr/bin/env bash
# 只读诊断 ROS 环境、话题类型和关键定位 TF，不启动节点或发送速度。
# ROS 环境脚本会读取若干可选变量，因此 source 完成前不能启用 set -u。
set -eo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
if [[ -f "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi
set -u

echo "ROS_DISTRO=${ROS_DISTRO:-unset} RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"
# 先输出系统级 ROS 报告，再列出现有通信图，便于区分环境问题和节点问题。
# ros2 doctor 在发现“有软件包可更新”等警告时也可能返回非零；诊断脚本仍应继续检查
# 话题和 TF，因此保留报告但不让单项警告中断后续步骤。
ros2 doctor --report || true
ros2 topic list -t || true
echo "关键 TF（3 秒采样）："
# tf2_echo 会持续输出。不要直接把它接到 head 等会提前关闭读取端的程序，否则 Jazzy 的
# ros2run 在汇报被终止进程时可能触发 BrokenPipeError，并被 Ubuntu 误报成 ros2 崩溃。
# 这里让 ROS 2 始终写入一个仍然打开的文件，超时结束后再显示有限内容。
tf_report="$(mktemp)"
trap 'rm -f "${tf_report}"' EXIT
timeout --signal=INT --kill-after=1 3 \
  ros2 run tf2_ros tf2_echo map base_link -r 1 >"${tf_report}" 2>&1 || true
sed -n '1,24p' "${tf_report}"
