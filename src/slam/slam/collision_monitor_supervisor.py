"""按安全顺序关闭 Nav2 Collision Monitor。

ROS 2 launch 在收到 Ctrl-C 时会近乎同时向所有子进程发送 SIGINT。Collision Monitor 的
进程信号清理可能正好与传感器/速度回调并发；Nav2 Jazzy 1.3.12 的该竞态会在
``PublisherBase::get_subscription_count()`` 中触发 SIGSEGV。

本监督进程不参与 ROS 通信，也不改变节点名、参数、remap 或生命周期：它只把所有命令行
参数原样转发给官方 ``collision_monitor``，并让子进程进入独立会话。退出时先留出很短的
排空窗口，让上游速度发布节点停止；随后由操作系统终止隔离的官方子进程，不再让该版本
进入已知有竞态的 rclcpp 信号清理路径。这样运行期仍使用原版 Nav2 碰撞保护，修复范围
只限于进程关闭方式；Collision Monitor 不保存地图或参数等持久状态，资源由内核回收。
"""

from __future__ import annotations

import ctypes
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Sequence

from ament_index_python.packages import get_package_prefix


# 速度平滑器为 20 Hz，0.25 s 覆盖五个发布周期；之后不再让已知有缺陷的官方信号处理器
# 执行资源销毁，而是结束隔离子进程并让内核回收进程私有资源。
DEFAULT_DRAIN_SECONDS = 0.25
PR_SET_PDEATHSIG = 1


def collision_monitor_executable() -> str:
    """通过 ament 索引定位系统安装的官方可执行文件，避免写死 ``/opt/ros``。"""
    prefix = Path(get_package_prefix("nav2_collision_monitor"))
    executable = prefix / "lib" / "nav2_collision_monitor" / "collision_monitor"
    if not executable.is_file():
        raise FileNotFoundError(f"Collision Monitor executable not found: {executable}")
    return str(executable)


def _drain_seconds() -> float:
    """读取内部排空时间；异常环境变量回退到经过回归测试的保守值。"""
    try:
        value = float(os.environ.get("WAKULA_COLLISION_DRAIN_SECONDS", DEFAULT_DRAIN_SECONDS))
    except ValueError:
        return DEFAULT_DRAIN_SECONDS
    return min(max(value, 0.0), 2.0)


def _isolate_child_and_arm_parent_death_signal() -> None:
    """建立独立会话，并保证监督器异常消失时子进程不会成为孤儿。

    launch 正常退出会走 ``supervise`` 的排空路径；若终端、IDE 或监督器被 SIGKILL，普通
    ``start_new_session`` 会让 Collision Monitor 被 systemd 收养并继续占用同名 ROS 节点。
    Linux ``PR_SET_PDEATHSIG`` 让内核在父进程消失时直接终止无状态子进程。
    """
    os.setsid()
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(PR_SET_PDEATHSIG, signal.SIGKILL, 0, 0, 0) != 0:
        error = ctypes.get_errno()
        raise OSError(error, os.strerror(error))


def supervise(command: Sequence[str], drain_seconds: float = DEFAULT_DRAIN_SECONDS) -> int:
    """运行官方节点，并在本进程收到终止信号后按“排空→隔离终止”顺序关闭。

    ``start_new_session`` 很关键：否则终端发给进程组的 Ctrl-C 也会绕过监督层直接到达
    Collision Monitor。信号处理器只设置标志，等待和子进程管理均在主循环完成，避免在
    Python 异步信号上下文里执行不可重入操作。
    """
    shutdown_signal: int | None = None

    def request_shutdown(signum, _frame):
        nonlocal shutdown_signal
        if shutdown_signal is None:
            shutdown_signal = signum

    previous_handlers = {
        signum: signal.signal(signum, request_shutdown)
        for signum in (signal.SIGINT, signal.SIGTERM)
    }
    child = subprocess.Popen(
        list(command), preexec_fn=_isolate_child_and_arm_parent_death_signal
    )
    try:
        while child.poll() is None and shutdown_signal is None:
            time.sleep(0.05)

        # 子节点自行退出时保留其返回码，便于 launch 正确报告真正的启动/参数错误。
        if shutdown_signal is None:
            return int(child.returncode or 0)

        # 此时上游节点已同时收到 launch 的 SIGINT。先让它们停止发布最后一帧速度；排空
        # 期间官方节点仍完整存活，不会出现半销毁 Publisher。
        deadline = time.monotonic() + max(0.0, drain_seconds)
        while child.poll() is None and time.monotonic() < deadline:
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

        if child.poll() is None:
            # 不能再发送 SIGINT/SIGTERM：两者都会进入 Jazzy 1.3.12 的同一 rclcpp 清理
            # 回调并可能在 PublisherBase::get_subscription_count() 中崩溃。子进程位于独立
            # 会话且没有持久数据；上游排空后直接结束它，DDS/文件描述符由内核可靠回收。
            try:
                child.kill()
            except ProcessLookupError:
                pass
        child.wait()
        # 这是监督器主动完成的预期关闭。不要把子进程的 SIGKILL 返回码传播给 launch，
        # 否则正常 Ctrl-C 会被误报成节点运行故障。
        return 0
    finally:
        for signum, handler in previous_handlers.items():
            signal.signal(signum, handler)


def main(args: Sequence[str] | None = None) -> int:
    """把 launch_ros 生成的 ROS 参数和 remap 原样交给 Collision Monitor。"""
    forwarded_args = list(sys.argv[1:] if args is None else args)
    command = [collision_monitor_executable(), *forwarded_args]
    return supervise(command, _drain_seconds())


if __name__ == "__main__":
    raise SystemExit(main())
