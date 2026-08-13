"""自主导航快捷入口的静态边界测试。"""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
WORKSPACE_ROOT = PACKAGE_ROOT.parents[1]


def test_root_auto_is_control_only():
    source = (WORKSPACE_ROOT / "auto").read_text(encoding="utf-8")
    assert '"/autonomy/toggle"' not in source  # shell 参数无需额外引号形式
    assert "/autonomy/toggle" in source
    assert "std_srvs/srv/Trigger" in source
    assert "launch_command=" not in source
    assert "exec ros2 launch" not in source
    assert "systemd-run" not in source
    assert "quadruped_gazebo" not in source


def test_keyboard_has_explicit_toggle_and_quit_keys():
    source = (PACKAGE_ROOT / "slam" / "autonomy_keyboard.py").read_text(encoding="utf-8")
    assert 'key in (" ", "t", "T")' in source
    assert 'key in ("q", "Q")' in source
    assert '"/autonomy/toggle"' in source
    assert '"/autonomy/state"' in source
    assert '"EXPLORING"' in source
    assert "WAITING_FOR_INPUTS（尚未开始移动" in source
