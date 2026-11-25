"""Native ROS executable를 고정 libtf2로 시작하고 실제 /proc mapping을 검사한다.

launch prefix에서 호출한다. 환경 source 순서에 의존하지 않게 LD_PRELOAD와
LD_LIBRARY_PATH를 이 process가 설정한다. native child의 실제 libtf2 mapping이
정확히 한 파일인지 확인하며 불일치/미로딩/child 실패는 launch 전체 실패가 된다.
SIGINT/TERM은 이 wrapper가 시작한 child에만 전달하고 wait/reap한다.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def main():
    library = Path(sys.argv[1]).resolve(strict=True)
    environment = os.environ.copy()
    environment["LD_PRELOAD"] = str(library)
    environment["LD_LIBRARY_PATH"] = str(library.parent) + ":" + environment.get("LD_LIBRARY_PATH", "")
    child = subprocess.Popen(sys.argv[2:], env=environment)

    def forward(number, _frame):
        if child.poll() is None:
            child.send_signal(number)

    signal.signal(signal.SIGINT, forward)
    signal.signal(signal.SIGTERM, forward)
    try:
        deadline = time.monotonic() + 10
        while child.poll() is None:
            mappings = (Path("/proc") / str(child.pid) / "maps").read_text()
            loaded = {
                Path(line.split()[-1]).resolve()
                for line in mappings.splitlines() if "/libtf2.so" in line
            }
            if loaded:
                if loaded != {library}:
                    raise RuntimeError(f"wrong libtf2 loaded: {loaded}, required={library}")
                print(f"TF2_LOADED pid={child.pid} library={library}", flush=True)
                return_code = child.wait()
                return return_code if return_code >= 0 else 128 - return_code
            if time.monotonic() >= deadline:
                raise RuntimeError("native process did not load required libtf2 within 10 s")
            time.sleep(0.02)
        return child.returncode or 1
    finally:
        if child.poll() is None:
            child.terminate()
        child.wait()


if __name__ == "__main__":
    sys.exit(main())
