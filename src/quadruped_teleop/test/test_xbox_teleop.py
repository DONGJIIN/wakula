"""Xbox 手柄纯状态机的安全行为回归测试。"""

import math

from quadruped_teleop.xbox_teleop import (
    apply_deadzone,
    button_pressed,
    safe_axis,
    TeleopConfig,
    XboxTeleopController,
)


def joy_state(*, axes=None, pressed=()):
    """生成常见 8 轴/11 键 Xbox 数组，方便测试只关注目标按键。"""
    axis_values = [0.0] * 8 if axes is None else list(axes)
    buttons = [0] * 11
    for index in pressed:
        buttons[index] = 1
    return axis_values, buttons


def test_deadzone_is_zero_and_rescales_remaining_travel():
    """中心漂移归零，满行程仍精确映射为 ±1。"""
    assert apply_deadzone(0.10, 0.12) == 0.0
    assert apply_deadzone(-0.12, 0.12) == 0.0
    assert apply_deadzone(1.0, 0.12) == 1.0
    assert apply_deadzone(-1.0, 0.12) == -1.0


def test_short_or_invalid_input_is_fail_safe():
    """驱动数组不足或含非法浮点数时不能产生运动。"""
    assert safe_axis([], 1) == 0.0
    assert safe_axis([math.nan], 0) == 0.0
    assert not button_pressed([], 4)
    result = XboxTeleopController(TeleopConfig()).update([], [])
    assert not result.active
    assert result.twist.linear.x == 0.0


def test_lb_must_be_held_to_generate_normal_speed_command():
    """摇杆回中按下 LB 解锁后，持续按住才按正常档生成速度。"""
    controller = XboxTeleopController(TeleopConfig(deadzone=0.0))
    axes, buttons = joy_state(axes=[0.5, 1.0, 0.0, -0.5, 0.0, 0.0, 0.0, 0.0])
    inactive = controller.update(axes, buttons)
    assert not inactive.active
    assert inactive.twist.linear.x == 0.0

    armed = controller.update(*joy_state(pressed=(4,)))
    assert armed.active
    assert armed.event == "teleop_armed"

    axes, buttons = joy_state(axes=axes, pressed=(4,))
    active = controller.update(axes, buttons)
    assert active.active
    assert active.twist.linear.x == 0.25
    assert active.twist.linear.y == 0.075
    assert active.twist.angular.z == -0.30


def test_a_x_y_select_speed_modes_on_button_edges():
    """A/X/Y 分别选择低速、正常、快速档，松开 LB 后仍保持选中档位。"""
    controller = XboxTeleopController(TeleopConfig(deadzone=0.0))
    controller.update(*joy_state(pressed=(0,)))
    controller.update(*joy_state())
    controller.update(*joy_state(pressed=(4,)))
    moving = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    slow = controller.update(*joy_state(axes=moving, pressed=(4,)))
    assert slow.speed_mode == "slow"
    assert slow.twist.linear.x == 0.12

    controller.update(*joy_state())
    controller.update(*joy_state(pressed=(3,)))
    controller.update(*joy_state())
    controller.update(*joy_state(pressed=(4,)))
    fast = controller.update(*joy_state(axes=moving, pressed=(4,)))
    assert fast.speed_mode == "fast"
    assert fast.twist.linear.x == 0.40


def test_off_center_lb_is_rejected_until_released_and_pressed_again():
    """带非零摇杆按下 LB 不得起步，回中后也必须松开并重新按下。"""
    controller = XboxTeleopController(TeleopConfig(deadzone=0.10))
    moving = [0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    rejected = controller.update(*joy_state(axes=moving, pressed=(4,)))
    assert not rejected.active
    assert rejected.event == "teleop_arm_rejected"

    still_held = controller.update(*joy_state(pressed=(4,)))
    assert not still_held.active
    controller.update(*joy_state())
    armed = controller.update(*joy_state(pressed=(4,)))
    assert armed.active


def test_timeout_requires_lb_release_before_rearming():
    """断流撤销使能；重连时持续按住 LB 也必须松开并再次安全按下。"""
    controller = XboxTeleopController(TeleopConfig(deadzone=0.0))
    controller.update(*joy_state(pressed=(4,)))
    moving = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert controller.update(*joy_state(axes=moving, pressed=(4,))).active

    controller.disarm_for_timeout()
    reconnected = controller.update(*joy_state(pressed=(4,)))
    assert not reconnected.active
    assert reconnected.event == "teleop_waiting_for_deadman_release"
    # 持续按住时不重复报告，避免节点按手柄帧率刷日志。
    assert controller.update(*joy_state(pressed=(4,))).event == ""

    controller.update(*joy_state())
    armed_again = controller.update(*joy_state(pressed=(4,)))
    assert armed_again.active


def test_axis_directions_and_lateral_limit_are_configurable():
    """不同驱动轴方向可翻转，横移速度按独立比例保守限制。"""
    config = TeleopConfig(
        deadzone=0.0,
        linear_x_direction=-1.0,
        linear_y_direction=-1.0,
        angular_z_direction=-1.0,
        lateral_speed_scale=0.4,
    )
    controller = XboxTeleopController(config)
    controller.update(*joy_state(pressed=(4,)))
    axes = [1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    result = controller.update(*joy_state(axes=axes, pressed=(4,)))
    assert result.twist.linear.x == -0.25
    assert result.twist.linear.y == -0.10
    assert result.twist.angular.z == -0.60


def test_b_latches_stop_until_safe_start_clear():
    """B 急停锁存；Start 仅在 LB 松开且摇杆回中时解除。"""
    controller = XboxTeleopController(TeleopConfig(deadzone=0.12))
    moving_axes = [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    stopped = controller.update(*joy_state(axes=moving_axes, pressed=(1, 4)))
    assert stopped.emergency_stop
    assert not stopped.active
    assert stopped.twist.linear.x == 0.0

    controller.update(*joy_state())
    rejected = controller.update(*joy_state(axes=moving_axes, pressed=(7,)))
    assert rejected.emergency_stop
    assert rejected.event == "emergency_stop_clear_rejected"

    controller.update(*joy_state())
    cleared = controller.update(*joy_state(pressed=(7,)))
    assert not cleared.emergency_stop
    assert cleared.event == "emergency_stop_cleared"


def test_stop_has_priority_when_b_and_start_arrive_together():
    """B 和 Start 同帧按下时必须保持急停，不能被清除动作覆盖。"""
    controller = XboxTeleopController(TeleopConfig())
    result = controller.update(*joy_state(pressed=(1, 7)))
    assert result.emergency_stop
    assert result.event == "emergency_stop_latched"


def test_reserved_buttons_do_not_change_motion_state():
    """RB、Back、Guide 和摇杆按下均为预留键，不能意外改变档位或使能。"""
    controller = XboxTeleopController(TeleopConfig())
    result = controller.update(*joy_state(pressed=(5, 6, 8, 9, 10)))
    assert result.speed_mode == "normal"
    assert not result.active
    assert not result.emergency_stop
    assert result.event == ""
