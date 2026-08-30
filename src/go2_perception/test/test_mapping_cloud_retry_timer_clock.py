"""Mapping cloud 재시도 timer의 clock 선택 계약을 검사한다."""

from enum import Enum
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from unittest.mock import patch


NODE_PATH = (
    Path(__file__).parents[1]
    / "go2_perception"
    / "mapping_cloud_accumulator_node.py"
)


class _ClockType(Enum):
    SYSTEM_TIME = 0
    ROS_TIME = 1
    STEADY_TIME = 2


class _Clock:
    def __init__(self, *, clock_type=_ClockType.SYSTEM_TIME) -> None:
        self.clock_type = clock_type


class _Node:
    def __init__(self, _name) -> None:
        self._node_clock = _Clock(clock_type=_ClockType.ROS_TIME)
        self.timer_clock = None
        self.timer_period = None

    def declare_parameter(self, _name, default):
        return SimpleNamespace(value=default)

    def create_publisher(self, *_args):
        return SimpleNamespace(publish=lambda _message: None)

    def create_subscription(self, *_args):
        return SimpleNamespace()

    def create_timer(self, period, _callback, *, clock=None):
        self.timer_period = period
        self.timer_clock = self._node_clock if clock is None else clock
        return SimpleNamespace()

    def get_logger(self):
        return SimpleNamespace(info=lambda _message: None)


class _QoSProfile:
    def __init__(self, **_kwargs) -> None:
        pass


class _Buffer:
    def __init__(self, **_kwargs) -> None:
        pass


class _TransformListener:
    def __init__(self, *_args) -> None:
        pass


class _RetryQueue:
    @classmethod
    def __class_getitem__(cls, _item):
        return cls

    def __init__(self, *_args) -> None:
        pass


class _Window:
    def __init__(self, *_args) -> None:
        pass


def _module(name, **attributes):
    module = ModuleType(name)
    for attribute_name, value in attributes.items():
        setattr(module, attribute_name, value)
    return module


def _ros_stubs():
    rclpy = _module("rclpy")
    return {
        "rclpy": rclpy,
        "rclpy.clock": _module(
            "rclpy.clock",
            Clock=_Clock,
            ClockType=_ClockType,
        ),
        "rclpy.duration": _module("rclpy.duration", Duration=SimpleNamespace),
        "rclpy.executors": _module(
            "rclpy.executors",
            ExternalShutdownException=RuntimeError,
        ),
        "rclpy.node": _module("rclpy.node", Node=_Node),
        "rclpy.qos": _module(
            "rclpy.qos",
            DurabilityPolicy=SimpleNamespace(VOLATILE=1),
            HistoryPolicy=SimpleNamespace(KEEP_LAST=1),
            QoSProfile=_QoSProfile,
            ReliabilityPolicy=SimpleNamespace(RELIABLE=1),
        ),
        "rclpy.time": _module("rclpy.time", Time=SimpleNamespace),
        "sensor_msgs": _module("sensor_msgs"),
        "sensor_msgs.msg": _module(
            "sensor_msgs.msg",
            PointCloud2=type("PointCloud2", (), {}),
        ),
        "tf2_ros": _module(
            "tf2_ros",
            Buffer=_Buffer,
            TransformException=RuntimeError,
            TransformListener=_TransformListener,
        ),
        "tf2_sensor_msgs": _module("tf2_sensor_msgs"),
        "tf2_sensor_msgs.tf2_sensor_msgs": _module(
            "tf2_sensor_msgs.tf2_sensor_msgs",
            do_transform_cloud=lambda cloud, _transform: cloud,
        ),
        "go2_perception": _module("go2_perception"),
        "go2_perception.mapping_cloud_accumulator": _module(
            "go2_perception.mapping_cloud_accumulator",
            MappingCloudRetryQueue=_RetryQueue,
            MappingCloudWindow=_Window,
            MappingCloudWindowError=RuntimeError,
            compact_xyz_cloud=lambda cloud: cloud,
            format_mapping_cloud_accounting=str,
        ),
        "go2_perception.mapping_cloud_gate_node": _module(
            "go2_perception.mapping_cloud_gate_node",
            OUTPUT_TOPIC="/go2_mapping/cloud_validated",
        ),
        "go2_perception.obstacle_candidate_node": _module(
            "go2_perception.obstacle_candidate_node",
            POINT_CLOUD_QOS=SimpleNamespace(),
        ),
    }


def test_retry_timer_uses_retained_steady_clock_when_node_uses_sim_time():
    module_name = "_mapping_cloud_accumulator_node_clock_contract"
    spec = importlib.util.spec_from_file_location(module_name, NODE_PATH)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)

    with patch.dict(sys.modules, _ros_stubs()):
        spec.loader.exec_module(module)
        node = module.MappingCloudAccumulatorNode()

    assert node.timer_period == module.RETRY_PERIOD_SECONDS
    assert node.timer_clock.clock_type is _ClockType.STEADY_TIME
    assert node._retry_clock is node.timer_clock
