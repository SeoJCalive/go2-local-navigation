"""실제 ROSClock의 역행 sequence로 공유 epoch callback과 mode 경계를 검사한다."""

from types import SimpleNamespace

from rclpy.clock import ROSClock
from rclpy.time import Time
from rclpy.clock import ClockType

from go2_inputs.simulation_epoch import watch_simulation_epoch


def test_backwards_clock_resets_once_and_forward_samples_continue():
    clock = ROSClock()
    clock._set_ros_time_is_active(True)
    resets = []
    warnings = []
    node = SimpleNamespace(
        get_parameter=lambda _: SimpleNamespace(value=True),
        get_clock=lambda: clock,
        get_logger=lambda: SimpleNamespace(warning=warnings.append),
    )
    handler = watch_simulation_epoch(node, lambda: resets.append(True))
    for stamp in (1, 2, 3, 1, 1, 2):
        clock.set_ros_time_override(Time(seconds=stamp, clock_type=ClockType.ROS_TIME))
    assert resets == [True]
    assert len(warnings) == 1
    handler.unregister()


def test_wall_time_node_does_not_register_a_reset_callback():
    node = SimpleNamespace(get_parameter=lambda _: SimpleNamespace(value=False))
    assert watch_simulation_epoch(node, lambda: None) is None
