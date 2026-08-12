"""按安全顺序关闭 Nav2 Collision Monitor。

ROS 2 launch 在收到 Ctrl-C 时会近乎同时向所有子进程发送 SIGINT。Collision Monitor 的
进程信号清理可能正好与传感器/速度回调并发；Nav2 Jazzy 1.3.12 的该竞态会在
``PublisherBase::get_subscription_count()`` 中触发 SIGSEGV。

本监督进程不参与 ROS 通信，也不改变节点名、参数、remap 或生命周期：它只把所有命令行
参数原样转发给官方 ``collision_monitor``，并让子进程进入独立会话。退出时先留出很短的
排空窗口，让上游速度发布节点停止；随后通过标准 ROS 2 生命周期服务串行执行 deactivate
和 cleanup，最后才向已清理的官方进程发送 SIGINT。这样运行期仍使用原版 Nav2 碰撞保护，
修复范围只限于进程关闭顺序。
"""

from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from typing import Sequence

from ament_index_python.packages import get_package_prefix


# 速度平滑器为 20 Hz，0.25 s 覆盖五个发布周期。真正的资源释放不再依赖等待时间，而由
# lifecycle deactivate/cleanup 串行完成。
DEFAULT_DRAIN_SECONDS = 0.25
CHILD_EXIT_TIMEOUT_SECONDS = 3.0
LIFECYCLE_COMMAND_TIMEOUT_SECONDS = 2.0


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


def request_lifecycle_transition(transition: str) -> bool:
    """请求标准生命周期转换；失败时返回 False，让调用者仍能完成兜底终止。

    使用 ROS 2 CLI 子进程可避免监督层自身加入 ROS executor 或接管子节点参数。CLI 与
    Collision Monitor 处于同一 ``ROS_DOMAIN_ID``，调用的仍是标准
    ``/collision_monitor/change_state`` 服务。
    """
    try:
        result = subprocess.run(
            ["ros2", "lifecycle", "set", "/collision_monitor", transition],
            check=False,
            capture_output=True,
            text=True,
            timeout=LIFECYCLE_COMMAND_TIMEOUT_SECONDS,
            start_new_session=True,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0 and "successful" in result.stdout.lower()


def supervise(command: Sequence[str], drain_seconds: float = DEFAULT_DRAIN_SECONDS) -> int:
    """运行官方节点，并在本进程收到终止信号后按“排空→SIGINT”顺序关闭。

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
    child = subprocess.Popen(list(command), start_new_session=True)
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
            # 生命周期服务回调由 Collision Monitor 的 executor 串行处理。成功 deactivate
            # 后停止运行期定时器，cleanup 再释放 Publisher/订阅；随后 SIGINT 的 preshutdown
            # 不会重复清理 active 资源。启动中途退出时转换可能被拒绝，仍走后面的进程兜底。
            deactivated = request_lifecycle_transition("deactivate")
            cleaned = deactivated and request_lifecycle_transition("cleanup")
            if deactivated and cleaned:
                print(
                    "[collision_monitor_supervisor] lifecycle resources cleaned before shutdown",
                    flush=True,
                )
            else:
                print(
                    "[collision_monitor_supervisor] lifecycle cleanup unavailable; "
                    "falling back to process shutdown",
                    file=sys.stderr,
                    flush=True,
                )

        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        try:
            return child.wait(timeout=CHILD_EXIT_TIMEOUT_SECONDS)
        except subprocess.TimeoutExpired:
            # 官方节点若在 DDS/驱动清理中卡住，先 TERM，最后才 KILL，防止残留进程污染
            # 下一次 launch。这个兜底只用于退出，不会在运行期触发。
            child.terminate()
            try:
                return child.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                child.kill()
                return child.wait()
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
