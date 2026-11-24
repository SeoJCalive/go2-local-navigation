"""ROS service와 실제 Launch/SIGINT로 clock 입력보다 앞선 종료를 검사한다.

가짜 lifecycle server는 프로토콜 경계만 제공한다. 성공 응답과 FINALIZED 상태가
다를 때의 거부를 확인하며, child의 READY/STOP 출력으로 signal 순서를 관찰한다.
simulator, Nav2 native node, 운동 command publisher는 실행하지 않는다.
"""
from pathlib import Path
import sys
from threading import Thread

import pytest
import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from launch_test_support import load
from go2_nav2.sim_session_guard import shutdown_managed_nodes


@pytest.mark.parametrize("mode", ["active", "lying_manager", "finalized"])
def test_shutdown_checks_finalized_after_manager_success(mode):
    groups = [("w07_nav", ["w07_planner"]), ("w07_loc", ["w07_amcl"])]
    states = {name: State.PRIMARY_STATE_ACTIVE for _, names in groups for name in names}
    if mode == "finalized":
        states = {name: State.PRIMARY_STATE_FINALIZED for name in states}
    calls = []
    context = Context()
    rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    server = rclpy.create_node("w07_shutdown_oracle", context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(server)

    def manage(kind, names):
        def callback(request, response):
            assert request.command == ManageLifecycleNodes.Request.SHUTDOWN
            calls.append(("manage", kind))
            for name in names:
                states[name] = (
                    State.PRIMARY_STATE_ACTIVE if mode == "lying_manager"
                    else State.PRIMARY_STATE_FINALIZED)
            response.success = True
            return response
        return callback

    def state(name):
        def callback(request, response):
            calls.append(("state", name))
            response.current_state.id = states[name]
            return response
        return callback

    for kind, names in groups:
        server.create_service(
            ManageLifecycleNodes, f"/lifecycle_manager_{kind}/manage_nodes", manage(kind, names))
        for name in names:
            server.create_service(GetState, f"/{name}/get_state", state(name))
    worker = Thread(target=executor.spin)
    worker.start()
    try:
        if mode == "lying_manager":
            with pytest.raises(RuntimeError):
                shutdown_managed_nodes(groups, timeout_sec=5.0)
            assert calls == [
                ("state", "w07_planner"), ("manage", "w07_nav"), ("state", "w07_planner")]
        else:
            result = shutdown_managed_nodes(groups, timeout_sec=5.0)
            assert result == {name: State.PRIMARY_STATE_FINALIZED for name in states}
            if mode == "finalized":
                assert calls == [("state", "w07_planner"), ("state", "w07_amcl")]
            else:
                assert calls == [
                    ("state", "w07_planner"), ("manage", "w07_nav"), ("state", "w07_planner"),
                    ("state", "w07_amcl"), ("manage", "w07_loc"), ("state", "w07_amcl")]
    finally:
        executor.shutdown(timeout_sec=2.0)
        worker.join(timeout=2.0)
        server.destroy_node()
        context.shutdown()
    assert not worker.is_alive()


def test_product_shutdown_handler_runs_before_clock_child_sigint(monkeypatch):
    from launch import LaunchContext, LaunchDescription, LaunchService
    from launch.actions import (
        EmitEvent, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction,
    )
    from launch.event_handlers import OnProcessExit, OnProcessIO
    from launch.events import Shutdown
    from go2_nav2 import sim_session_guard

    module = load("go2_mujoco_lidar", "go2_sim_navigation.launch.py")
    monkeypatch.setattr(module, "simulation_domain", lambda _: 41)
    monkeypatch.setattr(module, "simulator_world", lambda *args: (Path("/world.xml"), "fixture"))
    monkeypatch.setattr(module, "tf2_library", lambda _: Path("/tf2.so"))
    monkeypatch.setattr(module, "map_selection", lambda _: {"yaml": "/map.yaml"})
    monkeypatch.setattr(module, "session_lock", lambda _: object())
    events = []

    def shutdown(groups):
        assert [kind for kind, _ in groups] == ["navigation", "localization"]
        events.append("managed")
        return {}

    monkeypatch.setattr(sim_session_guard, "shutdown_managed_nodes", shutdown)
    context = LaunchContext()
    context.launch_configurations.update({
        "mode": "localization", "map": "/map.yaml", "initial_pose": "0,0,0",
        "simulator_pid": "123", "tf2_prefix": "/tf2", "horizontal_samples": "8",
        "vertical_samples": "2", "vertical_min": "-0.1", "vertical_max": "0.1",
    })
    actions = module._start(context)
    child = ExecuteProcess(cmd=[
        sys.executable, "-u", "-c",
        "import signal,sys;"
        "signal.signal(signal.SIGINT,lambda *a:(print('STOP',flush=True),sys.exit(0)));"
        "print('READY',flush=True);signal.pause()",
    ], output="screen", sigterm_timeout="2", sigkill_timeout="2")
    received = bytearray()
    requested = False

    def output(event):
        nonlocal requested
        received.extend(event.text)
        if b"STOP\n" in received:
            events.append("signal")
        if b"READY\n" in received and not requested:
            requested = True
            return [EmitEvent(event=Shutdown(reason="ready child"))]
        return []

    # Keep the product handler's position relative to the first input include.
    # Other ROS actions are not executed; service behavior is tested above.
    observed = []
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            observed.append(child)
        elif isinstance(action, RegisterEventHandler):
            observed.append(action)
    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnProcessIO(target_action=child, on_stdout=output)),
        RegisterEventHandler(OnProcessExit(target_action=child, on_exit=[
            EmitEvent(event=Shutdown(reason="owned child exited"))])),
        *observed,
        TimerAction(period=10.0, actions=[
            EmitEvent(event=Shutdown(reason="readiness deadline"))]),
    ]))
    assert service.run() == 0
    assert requested
    assert events == ["managed", "signal"]
