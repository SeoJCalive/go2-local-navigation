"""실제 subprocess로 parent 실패·고아 child·관찰 callback과 무잔류를 검증한다."""

import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import pytest

sys.path.insert(0, str(Path(__file__).parent / "acceptance"))
from owned_processes import OwnedProcesses


@pytest.mark.parametrize("ignore_term", [False, True])
def test_parent_exit_keeps_pumping_and_reaps_only_owned_session(ignore_term):
    ticks = []

    def pump():
        ticks.append(time.monotonic())

    manager = OwnedProcesses(pump)
    other = subprocess.Popen([sys.executable, "-c", "import signal; signal.pause()"],
                             start_new_session=True)
    process = manager.start([
        sys.executable, "-u", "-c",
        "import os, signal, sys;"
        "r,w=os.pipe();"
        "child=os.fork();"
        f"signal.signal(signal.SIGTERM, {'signal.SIG_IGN' if ignore_term else 'signal.SIG_DFL'}) if not child else None;"
        "os.write(w,b'1') if not child else os.read(r,1);"
        "signal.signal(signal.SIGINT, lambda *_: sys.exit(7));"
        "print(child, flush=True) if child else None;"
        "signal.pause()",
    ], stdout=subprocess.PIPE, text=True)
    try:
        ready, _, _ = select.select([process.stdout], [], [], 5)
        assert ready, "owned child PID event missing"
        child = int(process.stdout.readline())
        manager.observe()
        manager.stop(process)
        assert ticks
        assert process.returncode == 7
        assert signal.SIGTERM in manager.stops[-1]["signals"]
        assert (signal.SIGKILL in manager.stops[-1]["signals"]) is ignore_term
        assert not manager.remaining()
        assert not Path(f"/proc/{child}").exists()
        assert other.poll() is None
    finally:
        manager.stop(process)
        os.killpg(other.pid, signal.SIGTERM)
        other.wait(timeout=5)
