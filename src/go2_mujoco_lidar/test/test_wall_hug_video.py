"""녹화 명령·EOS 종료·선택 실행 연결을 GUI/ROS 없이 검사한다."""
import ast
import importlib.util
from pathlib import Path
import signal
import sys
import types

import pytest

HERE = Path(__file__).parent / "wall_hug"
spec = importlib.util.spec_from_file_location("record_video", HERE / "record_video.py")
video = importlib.util.module_from_spec(spec)
spec.loader.exec_module(video)


def test_recorder_command_and_eos(tmp_path):
    class Process:
        pid = 123
        returncode = None

        def poll(self):
            return self.returncode

        def send_signal(self, number):
            assert number == signal.SIGINT
            recorder.output.write_bytes(b"finalized-test-video")
            recorder.log.write("EOS received\n")
            self.returncode = 0

        def wait(self, timeout):
            assert timeout == 15
            return self.returncode

    class Owned:
        def start(self, argv, **kwargs):
            self.argv = argv
            return Process()

    owned, result = Owned(), {}
    original = video.VideoRecorder.cpu_seconds
    video.VideoRecorder.cpu_seconds = lambda self: 0
    try:
        recorder = video.VideoRecorder(tmp_path, ":77", owned, result)
        assert owned.argv[:5] == [
            "gst-launch-1.0", "-e", "ximagesrc", "display-name=:77", "use-damage=false"]
        for value in ("video/x-raw,framerate=10/1", "threads=1", "mp4mux"):
            assert value in owned.argv
        recorder.close()
        recorder.close()
        assert result["video"]["finalized"] is True
        with pytest.raises(FileExistsError):
            video.VideoRecorder(tmp_path, ":77", owned, {}, output=recorder.output)
    finally:
        video.VideoRecorder.cpu_seconds = original


@pytest.mark.parametrize("enabled", [False, True])
def test_runtime_flag_and_finalizer(enabled, monkeypatch):
    calls = []

    class Recorder:
        def __init__(self, *args):
            calls.append("start")

        def close(self):
            calls.append("EOS")

    monkeypatch.setitem(sys.modules, "record_video", types.SimpleNamespace(VideoRecorder=Recorder))
    template = '''capture_service=None
pad_process=None
def g02():
    global capture_service,pad_process
    capture_service=AsyncCapture(ROOT,g["display"],g["owned"],g["RESULT"])
try:
    g02()
finally:
    if capture_service is not None:
        capture_service.close()
'''
    module = ast.parse((HERE / "runtime_gui.py").read_text())
    # Execute the actual four maintained video transformations against a tiny runner.
    function = next(n for n in module.body if isinstance(n, ast.FunctionDef))
    start = next(i for i, n in enumerate(module.body)
                 if isinstance(n, ast.Assign) and any(
                     isinstance(t, ast.Name) and t.id == "source" for t in n.targets))
    transform = ast.Module(body=[function] + module.body[start + 1:start + 5], type_ignores=[])
    scope = {"source": template}
    exec(compile(transform, "video-transforms", "exec"), scope)
    result = {"errors": []}
    monkeypatch.setattr(sys, "argv", ["runtime_gui.py"] + (["--record-video"] if enabled else []))
    exec(scope["source"], {
        "ROOT": Path("/evidence"), "sys": sys,
        "AsyncCapture": lambda *args: types.SimpleNamespace(close=lambda: None),
        "g": {"display": ":77", "owned": object(), "RESULT": result}})
    assert calls == (["start", "EOS"] if enabled else [])
    assert result["errors"] == []
