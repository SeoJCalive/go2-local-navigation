"""수용 시험이 만든 POSIX session만 종료하고 PID/start tick으로 감사한다.

start_new_session으로 소유 경계를 만든다. parent가 먼저 종료해도 session의
child를 확인하고 group TERM/KILL을 적용한다. Linux subreaper로 고아 child도
직접 회수한다. 종료 대기 중 pump는 ROS observer callback을 계속 처리한다.
기존 Domain 1/42 프로세스 snapshot은 읽기 전용이며 signal 대상에 포함하지 않는다.
"""

import ctypes
import os
from pathlib import Path
import signal
import subprocess
import time


def snapshot():
    """현재 process identity와 session을 /proc에서 읽는다."""
    rows = {}
    for directory in Path("/proc").iterdir():
        if not directory.name.isdecimal():
            continue
        path = directory / "stat"
        try:
            fields = path.read_text().rsplit(")", 1)[1].split()
            rows[int(path.parent.name)] = {
                "state": fields[0], "session": int(fields[3]),
                "start_ticks": fields[19],
                "argv": path.with_name("cmdline").read_bytes().rstrip(b"\0").decode(
                    errors="replace").split("\0"),
            }
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    return rows


def existing_stack(project):
    """기존 Domain 1/42 app의 PID/start tick/argv를 보존한다."""
    result = {}
    for pid, row in snapshot().items():
        argv = row["argv"]
        try:
            environment = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            environment = []
        protected_domain = any(
            value in {b"ROS_DOMAIN_ID=1", b"ROS_DOMAIN_ID=42"} for value in environment)
        if protected_domain or any(value.startswith(str(project / "install") + "/") for value in argv) or (
            Path(argv[0]).name in {"unitree_mujoco", "go2_ctrl"}
            and any(argv[index:index + 2] in (
                ["-i", "1"], ["-d", "1"], ["-i", "42"], ["-d", "42"])
                    for index in range(len(argv) - 1))
        ) or any("/go2_nav2_mujoco_bridge/install/" in value for value in argv):
            result[str(pid)] = {"start_ticks": row["start_ticks"], "argv": argv}
    return result


class OwnedProcesses:
    """프로세스 생성·callback을 처리하는 종료·무잔류 확인을 한곳에서 수행한다."""

    def __init__(self, pump):
        if ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) != 0:
            raise OSError(ctypes.get_errno(), "PR_SET_CHILD_SUBREAPER")
        self.pump = pump
        self.processes = []
        self.identities = {}
        self.stops = []

    def observe(self):
        sessions = {process.pid for process in self.processes}
        for pid, row in snapshot().items():
            if row["session"] in sessions:
                self.identities[pid] = row

    def start(self, command, **kwargs):
        process = subprocess.Popen(command, start_new_session=True, **kwargs)
        self.processes.append(process)
        self.observe()
        return process

    def stop(self, process):
        began = time.monotonic()
        sent = []
        self.observe()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            sent.append(int(signal.SIGINT))
        phases = [(None, 15), (signal.SIGTERM, 10), (signal.SIGKILL, 5)]
        for action, timeout in phases:
            current = snapshot()
            members = {pid: row for pid, row in current.items()
                       if row["session"] == process.pid}
            if not members:
                break
            if action is not None:
                try:
                    os.killpg(process.pid, action)
                    sent.append(int(action))
                except ProcessLookupError:
                    pass
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                self.observe()
                process.poll()
                # poll() owns the Popen parent; waitpid owns adopted descendants.
                for pid in self.identities:
                    if pid != process.pid:
                        try:
                            os.waitpid(pid, os.WNOHANG)
                        except ChildProcessError:
                            pass
                if not any(row["session"] == process.pid for row in snapshot().values()):
                    break
                self.pump()
        process.poll()
        remaining = {pid: row for pid, row in snapshot().items()
                     if row["session"] == process.pid}
        self.stops.append({"pid": process.pid, "wall_sec": time.monotonic() - began,
                           "remaining": sorted(remaining), "signals": sent})
        if remaining:
            raise RuntimeError(f"owned session {process.pid} survived: {remaining}")
        return process.returncode

    def remaining(self):
        self.observe()
        current = snapshot()
        sessions = {process.pid for process in self.processes}
        return sorted(pid for pid, row in current.items()
                      if row["session"] in sessions or (
                          pid in self.identities
                          and row["start_ticks"] == self.identities[pid]["start_ticks"]))
