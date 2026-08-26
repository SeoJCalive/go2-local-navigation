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


class _FakeAdapterNode:
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
    publisher_module = ModuleType("rclpy.publisher")
    publisher_module.Publisher = type("Publisher", (), {})

    geometry_msgs_module = ModuleType("geometry_msgs")
    geometry_msgs_msg_module = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg_module.Twist = type("Twist", (), {})
    unitree_api_module = ModuleType("unitree_api")
    unitree_api_msg_module = ModuleType("unitree_api.msg")
    unitree_api_msg_module.Request = type("Request", (), {})

    for module_name, module in (
        ("rclpy", rclpy_module),
        ("rclpy.executors", executors_module),
        ("rclpy.node", node_module),
        ("rclpy.publisher", publisher_module),
        ("geometry_msgs", geometry_msgs_module),
        ("geometry_msgs.msg", geometry_msgs_msg_module),
        ("unitree_api", unitree_api_module),
        ("unitree_api.msg", unitree_api_msg_module),
    ):
        monkeypatch.setitem(sys.modules, module_name, module)


def test_given_inactive_context_when_interrupted_then_does_not_log(monkeypatch) -> None:
    runtime = _FakeRuntime()
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(sys.modules, "go2_control.motion_adapter_node", raising=False)
    adapter_module = importlib.import_module("go2_control.motion_adapter_node")
    node = _FakeAdapterNode()
    monkeypatch.setattr(adapter_module, "MotionAdapterNode", lambda: node)

    adapter_module.main()

    assert node.destroyed
    assert runtime.shutdown_call_count == 0
    assert node.log_messages == []
