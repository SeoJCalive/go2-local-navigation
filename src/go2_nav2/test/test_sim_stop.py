"""Stop sends launch SIGINT only after independently confirmed FINALIZED."""
from pathlib import Path
import shutil
import signal

import pytest
from go2_nav2 import sim_stop


def trial(monkeypatch, tmp_path):
    proc = tmp_path / "proc"
    proc.mkdir()
    (proc / "cmdline").write_bytes(b"ros2\0launch\0go2_sim_navigation.launch.py\0")
    (proc / "environ").write_bytes(b"ROS_DOMAIN_ID=41\0")
    fields = ["0"] * 20
    fields[0], fields[19] = "S", "1234"
    (proc / "stat").write_text("999 (launch) " + " ".join(fields))
    monkeypatch.setattr(sim_stop, "Path", lambda _: proc)
    monkeypatch.setenv("ROS_DOMAIN_ID", "41")
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "1")
    return proc


def test_finalized_precedes_signal(monkeypatch, tmp_path):
    proc = trial(monkeypatch, tmp_path)
    events = []
    def managed(groups):
        events.append(("finalized", [g[0] for g in groups]))
        return {"planner_server": 4}
    def kill(pid, sig):
        events.append(("signal", pid, sig))
        shutil.rmtree(proc)
    monkeypatch.setattr(sim_stop, "shutdown_managed_nodes", managed)
    monkeypatch.setattr(sim_stop.os, "kill", kill)
    assert sim_stop.stop(999) == {"planner_server": 4}
    assert events == [("finalized", ["navigation", "localization"]),
                      ("signal", 999, signal.SIGINT)]


def test_failed_lifecycle_preserves_clock(monkeypatch, tmp_path):
    trial(monkeypatch, tmp_path)
    signals = []
    def failure(groups):
        raise TimeoutError("lifecycle")
    monkeypatch.setattr(sim_stop, "shutdown_managed_nodes", failure)
    monkeypatch.setattr(sim_stop.os, "kill", lambda *a: signals.append(a))
    with pytest.raises(TimeoutError):
        sim_stop.stop(999)
    assert not signals


def test_other_domain_pid_rejected(monkeypatch, tmp_path):
    proc = trial(monkeypatch, tmp_path)
    (proc / "environ").write_bytes(b"ROS_DOMAIN_ID=1\0")
    with pytest.raises(ValueError, match="differ"):
        sim_stop.stop(999)


def test_xml_without_cyclone_rmw_is_rejected(monkeypatch, tmp_path):
    trial(monkeypatch, tmp_path)
    monkeypatch.setenv("ROS_LOCALHOST_ONLY", "0")
    monkeypatch.setenv("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    monkeypatch.setattr(sim_stop, "cyclonedds_network_matches", lambda *a: True)
    with pytest.raises(ValueError, match="loopback"):
        sim_stop.stop(999)
