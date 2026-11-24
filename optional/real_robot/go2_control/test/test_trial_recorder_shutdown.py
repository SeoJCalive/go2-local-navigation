import importlib
import sys
from types import ModuleType


class _ExternalShutdownException(Exception):
    pass


class _ExecutorConversionError(RuntimeError):
    pass


class _FakeRuntime:
    def __init__(self, *, context_active_after_error: bool) -> None:
        self.active = True
        self.context_active_after_error = context_active_after_error
        self.shutdown_call_count = 0

    def init(self, *, args: list[str] | None = None) -> None:
        del args

    def spin(self, node) -> None:
        del node
        self.active = self.context_active_after_error
        raise _ExecutorConversionError(
            "Unable to convert call argument to Python object"
        )

    def ok(self) -> bool:
        return self.active

    def shutdown(self) -> None:
        self.shutdown_call_count += 1
        self.active = False


class _FakeRecorderNode:
    def __init__(self) -> None:
        self.destroyed = False
        self.write_call_count = 0

    def write_record(self) -> None:
        self.write_call_count += 1

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

    geometry_msgs_module = ModuleType("geometry_msgs")
    geometry_msgs_msg_module = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg_module.Twist = type("Twist", (), {})
    nav_msgs_module = ModuleType("nav_msgs")
    nav_msgs_msg_module = ModuleType("nav_msgs.msg")
    nav_msgs_msg_module.Odometry = type("Odometry", (), {})
    unitree_api_module = ModuleType("unitree_api")
    unitree_api_msg_module = ModuleType("unitree_api.msg")
    unitree_api_msg_module.Request = type("Request", (), {})

    for module_name, module in (
        ("rclpy", rclpy_module),
        ("rclpy.executors", executors_module),
        ("rclpy.node", node_module),
        ("geometry_msgs", geometry_msgs_module),
        ("geometry_msgs.msg", geometry_msgs_msg_module),
        ("nav_msgs", nav_msgs_module),
        ("nav_msgs.msg", nav_msgs_msg_module),
        ("unitree_api", unitree_api_module),
        ("unitree_api.msg", unitree_api_msg_module),
    ):
        monkeypatch.setitem(sys.modules, module_name, module)


def test_given_inactive_context_when_executor_conversion_fails_then_recorder_exits_cleanly(
    monkeypatch,
) -> None:
    runtime = _FakeRuntime(context_active_after_error=False)
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(
        sys.modules,
        "go2_control.trial_recorder_node",
        raising=False,
    )
    recorder_module = importlib.import_module(
        "go2_control.trial_recorder_node"
    )
    node = _FakeRecorderNode()
    monkeypatch.setattr(recorder_module, "TrialRecorderNode", lambda: node)

    recorder_module.main()

    assert node.write_call_count == 1
    assert node.destroyed
    assert runtime.shutdown_call_count == 0


def test_given_active_context_when_executor_conversion_fails_then_error_is_visible(
    monkeypatch,
) -> None:
    runtime = _FakeRuntime(context_active_after_error=True)
    _install_ros_stubs(monkeypatch, runtime)
    monkeypatch.delitem(
        sys.modules,
        "go2_control.trial_recorder_node",
        raising=False,
    )
    recorder_module = importlib.import_module(
        "go2_control.trial_recorder_node"
    )
    node = _FakeRecorderNode()
    monkeypatch.setattr(recorder_module, "TrialRecorderNode", lambda: node)

    raised_error = None
    try:
        recorder_module.main()
    except RuntimeError as error:
        raised_error = error

    assert node.write_call_count == 1
    assert node.destroyed
    assert runtime.shutdown_call_count == 1
    assert raised_error is not None
    assert str(raised_error) == "Unable to convert call argument to Python object"
