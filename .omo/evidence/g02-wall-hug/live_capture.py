"""전용 Xvfb 주기 촬영 child. 완료된 PNG만 live와 latest에 게시한다."""
import datetime
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

root = Path(sys.argv[1])
while True:
    step = (root / "step.txt").read_text().strip()
    name = datetime.datetime.now().strftime("%H%M%S") + "_" + step + ".png"
    output = root / "live" / name
    temporary = output.with_name("."+output.stem+".capturing.png")
    run = subprocess.run(
        ["import", "-display", os.environ["DISPLAY"], "-window", "root",
         "-resize", "1280x>", str(temporary)], timeout=15, capture_output=True)
    if run.returncode == 0:
        os.replace(temporary,output)
        latest=root/"live"/f".latest-{os.getpid()}.png"
        shutil.copyfile(output,latest)
        os.replace(latest,root/"live/latest.png")
    else:
        print(run.stderr.decode(), flush=True)
    time.sleep(20)
