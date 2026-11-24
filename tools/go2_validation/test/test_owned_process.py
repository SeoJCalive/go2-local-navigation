"""다른 ROS process는 보존하고 실패한 owned session의 child까지 종료한다."""

from pathlib import Path
import ctypes
import os
import subprocess
import sys
import time

from go2_validation.preflight_host import scan_residual_processes, stop_owned_process


def test_residual_scan_uses_session_identity(tmp_path):
    for pid, session, state in ((10, 10, "S"), (11, 10, "S"), (12, 12, "S"), (13, 10, "Z")):
        path = tmp_path / str(pid)
        path.mkdir()
        (path / "stat").write_text(f"{pid} (map_server) {state} 1 {session} {session} 0")
        (path / "cmdline").write_bytes(b"map_server\0")
    residual = scan_residual_processes((10,), tmp_path)
    assert residual == ("10:map_server", "11:map_server")


def test_parent_failure_still_cleans_child_and_preserves_unrelated_session(tmp_path):
    libc = ctypes.CDLL(None)
    was_subreaper = ctypes.c_int()
    assert libc.prctl(37, ctypes.byref(was_subreaper), 0, 0, 0) == 0
    assert libc.prctl(36, 1, 0, 0, 0) == 0
    child_pid_file = tmp_path / "child.pid"
    unrelated = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(60)", "map_server"],
        start_new_session=True,
    )
    parent = subprocess.Popen([
        sys.executable, "-c",
        "import subprocess,sys; "
        "p=subprocess.Popen([sys.executable,'-c',"
        "'import os,signal,time; from pathlib import Path; "
        "signal.signal(signal.SIGTERM,signal.SIG_IGN); "
        "Path('+repr(sys.argv[1])+').write_text(str(os.getpid())); time.sleep(60)']); sys.exit(7)",
        str(child_pid_file),
    ], start_new_session=True)
    try:
        assert parent.wait(timeout=10) == 7
        deadline = time.monotonic() + 10
        while not child_pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.01)
        child_pid = int(child_pid_file.read_text())
        assert scan_residual_processes((parent.pid,))
        assert stop_owned_process(parent) == 7
        assert not scan_residual_processes((parent.pid,))
        assert unrelated.poll() is None
        assert os.waitpid(child_pid, 0)[0] == child_pid
        assert not Path(f"/proc/{child_pid}").exists()
    finally:
        stop_owned_process(parent)
        stop_owned_process(unrelated)
        assert libc.prctl(36, was_subreaper.value, 0, 0, 0) == 0
