"""목적: G02의 전용 DISPLAY 전체 화면을 저부하 H.264 MP4로 선택 녹화한다.

흐름: owned registry로 gst-launch -e 실행 -> SIGINT -> EOS/mux 완료 확인.
입력: 증거 root, harness DISPLAY, OwnedProcesses. 출력: live/*.mp4와 result video.
기본 10 fps/ultrafast/zerolatency/encoder thread 1. GUI·ROS·주행은 시작하지 않는다.
CLI는 --display/--output/--duration으로 이미 실행 중인 X 화면만 녹화한다.
"""
import argparse
import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


class VideoRecorder:
    def __init__(self, root, display, owned, result, output=None):
        self.owned, self.result = owned, result
        root = Path(root)
        self.output = Path(output) if output else root / "live" / (
            f"video-{datetime.datetime.now():%Y%m%d-%H%M%S}-{os.getpid()}.mp4")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        if self.output.exists():
            raise FileExistsError(self.output)
        self.log_path = self.output.with_suffix(".gst.log")
        self.log = self.log_path.open("w")
        self.argv = [
            "gst-launch-1.0", "-e", "ximagesrc", f"display-name={display}",
            "use-damage=false", "show-pointer=false", "!",
            "video/x-raw,framerate=10/1", "!", "videoconvert", "!",
            "video/x-raw,format=I420", "!", "x264enc", "speed-preset=ultrafast",
            "tune=zerolatency", "threads=1", "bitrate=1500", "key-int-max=100",
            "!", "mp4mux", "!", "filesink", "location=" + json.dumps(str(self.output))]
        self.process = owned.start(self.argv, stdout=self.log, stderr=self.log)
        self.started = time.monotonic()
        self.cpu_start = self.cpu_seconds()
        result.setdefault("pids", []).append({
            "name": "video-recorder", "pid": self.process.pid, "argv": self.argv})

    def cpu_seconds(self):
        fields = Path(f"/proc/{self.process.pid}/stat").read_text().rsplit(")", 1)[1].split()
        return (int(fields[11]) + int(fields[12])) / os.sysconf("SC_CLK_TCK")

    def close(self):
        if self.log.closed:
            return
        elapsed = time.monotonic() - self.started
        cpu = None
        try:
            if self.process.poll() is None:
                cpu = (self.cpu_seconds() - self.cpu_start) / elapsed * 100
                # -e turns SIGINT into EOS and waits for mp4mux to write its index.
                self.process.send_signal(signal.SIGINT)
            self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.owned.stop(self.process)
            raise RuntimeError("video recorder exceeded EOS deadline")
        finally:
            self.log.close()
        text = self.log_path.read_text()
        finalized = (self.process.returncode == 0 and "EOS" in text
                     and self.output.is_file() and self.output.stat().st_size > 0)
        self.result["video"] = {
            "path": str(self.output), "wall_s": elapsed,
            "cpu_percent_one_core": cpu, "finalized": finalized,
            "returncode": self.process.returncode}
        if not finalized:
            raise RuntimeError(f"video did not finalize: {self.log_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--display", required=True)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--duration", type=float, default=30)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "acceptance"))
    from owned_processes import OwnedProcesses
    done = threading.Event()
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: done.set())
    owned = OwnedProcesses(lambda: done.wait(.02))
    result = {}
    recorder = VideoRecorder(args.output.parent, args.display, owned, result, args.output)
    try:
        done.wait(args.duration)
    finally:
        try:
            recorder.close()
        finally:
            owned.stop(recorder.process)
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
