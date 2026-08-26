import importlib
import sys
from types import ModuleType


class _ExternalShutdownException(Exception):
    pass


class _ExecutorConversionError(RuntimeError):
    pass


class _FakeRuntime:
    def __init__(
        self,
        *,
        keyboard_interrupt: bool = False,
        runtime_error: bool = False,
        deactivate_before_error: bool = True,
    ) -> None:
        self.active = True
        self.shutdown_call_count = 0
        self.keyboard_interrupt = keyboard_interrupt
        self.runtime_error = runtime_error
        self.deactivate_before_error = deactivate_before_error

    def init(self, *, args: list[str] | None = None) -> None:
        del args

    def spin(self, node) -> None:
        del node
        if self.runtime_error:
            if self.deactivate_before_error:
                self.active = False
            raise _ExecutorConversionError(
                "Unable to convert call argument to Python object"
            )
        self.active = False
        if self.keyboard_interrupt:
            raise KeyboardInterrupt
        raise _ExternalShutdownException

    def ok(self) -> bool:
        return self.active

    def shutdown(self) -> None:
        self.shutdown_call_count += 1


class _FakeLogger:
    def __init__(self, messages: list[str]) -> None:
        self._messages = messages

    def info(self, message: str) -> None:
        self._messages.append(message)


class _FakeAdapterNode:
    def __init__(self) -> None:
        self.destroyed = False
        self.log_messages: list[str] = []
        self._received_sample_count = 0
        self._seen_warning_codes: set[str] = set()

    @property
    def published_sample_count(self) -> int:
        return 0

    @property
    def rejected_sample_count(self) -> int:
        return 0

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
    qos_module.DurabilityPolicy = type("DurabilityPolicy", (), {"VOLATILE": "volatile"})
    qos_module.HistoryPolicy = type("HistoryPolicy", (), {"KEEP_LAST": "keep_last"})
    qos_module.QoSProfile = type("QoSProfile", (), {"__init__": lambda self, **kwargs: None})
    qos_module.ReliabilityPolicy = type("ReliabilityPolicy", (), {"RELIABLE": "reliable"})

    geometry_msgs_module = ModuleType("geometry_msgs")
    geometry_msgs_msg_module = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg_module.TransformStamped = type("TransformStamped", (), {})
    nav_msgs_module = ModuleType("nav_msgs")
    nav_msgs_msg_module = ModuleType("nav_msgs.msg")
    nav_msgs_msg_module.Odometry = type("Odometry", (), {})
    tf2_ros_module = ModuleType("tf2_ros")
    tf2_ros_module.TransformBroadcaster = type("TransformBroadcaster", (), {})

    for module_name, module in (
        ("rclpy", rclpy_module),
        ("rclpy.executors", executors_module),
        ("rclpy.node", node_module),
        ("rclpy.qos", qos_module),
        ("geometry_msgs", geometry_msgs_module),
        ("geometry_msgs.msg", geometry_msgs_msg_module),
        ("nav_msgs", nav_msgs_module),
        ("nav_msgs.msg", nav_msgs_msg_module),
        ("tf2_ros", tf2_ros_module),
    ):
        monkeypatch.setitem(sys.modules, module_name, module)


def test_given_external_shutdown_when_adapter_exits_then_does_not_shutdown_twice(monkeypatch) -> None:
    runtime = _FakeRuntime()
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(sys.modules, "go2_state_estimation.odometry_adapter_node", raising=False)
    adapter_module = importlib.import_module("go2_state_estimation.odometry_adapter_node")
    node = _FakeAdapterNode()
    monkeypatch.setattr(adapter_module, "OdometryAdapterNode", lambda: node)

    external_shutdown_raised = False
    try:
        adapter_module.main()
    except _ExternalShutdownException:
        external_shutdown_raised = True

    assert node.destroyed
    assert runtime.shutdown_call_count == 0
    assert not external_shutdown_raised
    assert node.log_messages == []


def test_given_inactive_context_when_interrupted_then_adapter_does_not_log(
    monkeypatch,
) -> None:
    runtime = _FakeRuntime(keyboard_interrupt=True)
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(
        sys.modules,
        "go2_state_estimation.odometry_adapter_node",
        raising=False,
    )
    adapter_module = importlib.import_module(
        "go2_state_estimation.odometry_adapter_node"
    )
    node = _FakeAdapterNode()
    monkeypatch.setattr(adapter_module, "OdometryAdapterNode", lambda: node)

    adapter_module.main()

    assert node.destroyed
    assert runtime.shutdown_call_count == 0
    assert node.log_messages == []


def test_given_inactive_context_when_executor_conversion_fails_then_adapter_exits_cleanly(
    monkeypatch,
) -> None:
    runtime = _FakeRuntime(runtime_error=True)
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(
        sys.modules,
        "go2_state_estimation.odometry_adapter_node",
        raising=False,
    )
    adapter_module = importlib.import_module(
        "go2_state_estimation.odometry_adapter_node"
    )
    node = _FakeAdapterNode()
    monkeypatch.setattr(adapter_module, "OdometryAdapterNode", lambda: node)

    adapter_module.main()

    assert node.destroyed
    assert runtime.shutdown_call_count == 0


def test_given_active_context_when_executor_conversion_fails_then_error_is_visible(
    monkeypatch,
) -> None:
    runtime = _FakeRuntime(
        runtime_error=True,
        deactivate_before_error=False,
    )
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(
        sys.modules,
        "go2_state_estimation.odometry_adapter_node",
        raising=False,
    )
    adapter_module = importlib.import_module(
        "go2_state_estimation.odometry_adapter_node"
    )
    node = _FakeAdapterNode()
    monkeypatch.setattr(adapter_module, "OdometryAdapterNode", lambda: node)

    raised_error = None
    try:
        adapter_module.main()
    except RuntimeError as error:
        raised_error = error

    assert node.destroyed
    assert raised_error is not None
    assert str(raised_error) == "Unable to convert call argument to Python object"
