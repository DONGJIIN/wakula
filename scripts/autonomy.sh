#!/usr/bin/env bash
set -eo pipefail

# 运行中一键启停自主任务。STOP 只取消自动任务和越障交接，不关闭传感器、SLAM 或 RViz，
# 因而可以检查现场状态后再次 START。关闭整套 launch 仍使用启动终端中的 Ctrl-C。
source /opt/ros/jazzy/setup.bash
source "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install/setup.bash"
# ROS 环境脚本会读取可选变量；完成 source 后再启用 nounset。
set -u

case "${1:-status}" in
  start)
    ros2 service call /autonomy/set_enabled std_srvs/srv/SetBool "{data: true}"
    ;;
  stop)
    ros2 service call /autonomy/set_enabled std_srvs/srv/SetBool "{data: false}"
    ;;
  toggle)
    ros2 service call /autonomy/toggle std_srvs/srv/Trigger "{}"
    ;;
  status)
    ros2 topic echo --once /autonomy/state
    ;;
  *)
    echo "用法: $0 {toggle|start|stop|status}" >&2
    exit 2
    ;;
esac
