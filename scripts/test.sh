#!/usr/bin/env bash
# 运行 colcon 测试并汇总失败详情；额外参数可限制测试包。
set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
cd "$workspace_dir"
# console_direct+ 实时显示 pytest/launch_test 输出，CI 与本地行为保持一致。
colcon test --event-handlers console_direct+ "$@"
colcon test-result --verbose
