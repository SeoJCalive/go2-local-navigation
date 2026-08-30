import importlib
import sys
from types import ModuleType


class _ExternalShutdownException(Exception):
    pass


class _FakeRuntime:
    def __init__(self) -> None:
        self.active = True
        self.shutdown_call_count = 0

    def init(self, *, args: list[str] | None = None) -> None:
        del args

    def spin(self, node) -> None:
        del node
        self.active = False
        raise KeyboardInterrupt

    def ok(self) -> bool:
        return self.active

    def shutdown(self) -> None:
        self.shutdown_call_count += 1


class _FakeLogger:
    def __init__(self, messages: list[str]) -> None:
        self._messages = messages

    def info(self, message: str) -> None:
        self._messages.append(message)


class _FakeNode:
    def __init__(self) -> None:
        self.destroyed = False
        self.log_messages: list[str] = []

    def get_logger(self) -> _FakeLogger:
        return _FakeLogger(self.log_messages)

    def destroy_node(self) -> None:
        self.destroyed = True


def _install_ros_stubs(monkeypatch, runtime: _FakeRuntime) -> None:
    rclpy_module = ModuleType("rclpy")
    rclpy_module.init = runtime.init
    rclpy_module.ok = runtime.ok
    rclpy_module.shutdown = runtime.shutdown
    rclpy_module.spin = runtime.spin

    executors_module = ModuleType("rclpy.executors")
    executors_module.ExternalShutdownException = _ExternalShutdownException
    node_module = ModuleType("rclpy.node")
    node_module.Node = type("Node", (), {})
    qos_module = ModuleType("rclpy.qos")
    qos_module.DurabilityPolicy = type(
        "DurabilityPolicy",
        (),
        {"VOLATILE": "volatile"},
    )
    qos_module.HistoryPolicy = type(
        "HistoryPolicy",
        (),
        {"KEEP_LAST": "keep_last"},
    )
    qos_module.QoSProfile = type(
        "QoSProfile",
        (),
        {"__init__": lambda self, **kwargs: None},
    )
    qos_module.ReliabilityPolicy = type(
        "ReliabilityPolicy",
        (),
        {"RELIABLE": "reliable"},
    )
    time_module = ModuleType("rclpy.time")
    time_module.Time = type("Time", (), {})

    geometry_module = ModuleType("geometry_msgs")
    geometry_msg_module = ModuleType("geometry_msgs.msg")
    geometry_msg_module.TransformStamped = type("TransformStamped", (), {})
    sensor_module = ModuleType("sensor_msgs")
    sensor_msg_module = ModuleType("sensor_msgs.msg")
    sensor_msg_module.PointCloud2 = type("PointCloud2", (), {})
    sensor_py_module = ModuleType("sensor_msgs_py")
    point_cloud_module = ModuleType("sensor_msgs_py.point_cloud2")
    sensor_py_module.point_cloud2 = point_cloud_module
    std_module = ModuleType("std_msgs")
    std_msg_module = ModuleType("std_msgs.msg")
    std_msg_module.Header = type("Header", (), {})
    tf_module = ModuleType("tf2_ros")
    tf_module.Buffer = type("Buffer", (), {})
    tf_module.TransformException = type("TransformException", (Exception,), {})
    tf_module.TransformListener = type("TransformListener", (), {})

    for module_name, module in (
        ("rclpy", rclpy_module),
        ("rclpy.executors", executors_module),
        ("rclpy.node", node_module),
        ("rclpy.qos", qos_module),
        ("rclpy.time", time_module),
        ("geometry_msgs", geometry_module),
        ("geometry_msgs.msg", geometry_msg_module),
        ("sensor_msgs", sensor_module),
        ("sensor_msgs.msg", sensor_msg_module),
        ("sensor_msgs_py", sensor_py_module),
        ("sensor_msgs_py.point_cloud2", point_cloud_module),
        ("std_msgs", std_module),
        ("std_msgs.msg", std_msg_module),
        ("tf2_ros", tf_module),
    ):
        monkeypatch.setitem(sys.modules, module_name, module)


def test_given_inactive_context_when_interrupted_then_node_does_not_log(
    monkeypatch,
) -> None:
    runtime = _FakeRuntime()
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(
        sys.modules,
        "go2_perception.obstacle_candidate_node",
        raising=False,
    )
    obstacle_module = importlib.import_module(
        "go2_perception.obstacle_candidate_node"
    )
    node = _FakeNode()
    monkeypatch.setattr(obstacle_module, "ObstacleCandidateNode", lambda: node)

    obstacle_module.main()

    assert node.destroyed
    assert runtime.shutdown_call_count == 0
    assert node.log_messages == []


def test_given_inactive_context_when_mapping_gate_is_interrupted_then_it_exits_cleanly(
    monkeypatch,
) -> None:
    runtime = _FakeRuntime()
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(
        sys.modules,
        "go2_perception.mapping_cloud_gate_node",
        raising=False,
    )
    mapping_module = importlib.import_module(
        "go2_perception.mapping_cloud_gate_node"
    )
    node = _FakeNode()
    monkeypatch.setattr(mapping_module, "MappingCloudGateNode", lambda: node)

    mapping_module.main()

    assert node.destroyed
    assert runtime.shutdown_call_count == 0
    assert node.log_messages == []
