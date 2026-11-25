"""Independent milestone capture process; the caller only sends a small packet.

The existing process registry owns the worker's POSIX session and its ImageMagick
descendants. All registry mutations stay in the parent. A separate socket carries
ordered requests; completed PNGs and completion records are published atomically.
The pad driver never calls or waits for this worker.
"""
import datetime
import json
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import time


class AsyncCapture:
    def __init__(self,root,display,owned,result):
        self.root=Path(root)
        self.owned=owned
        self.result=result
        self.sequence=0
        self.last_loop_ns=None
        self.records=self.root/"capture-completions.jsonl"
        self.records.write_text("")
        self.log=(self.root/"capture-worker.log").open("w")
        self.channel,child=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
        argv=[sys.executable,__file__,"--worker",str(child.fileno()),str(root),display]
        self.process=owned.start(argv,pass_fds=(child.fileno(),),
            stdout=self.log,stderr=self.log)
        child.close()
        self.channel.setblocking(False)
        result.setdefault("pids",[]).append({
            "name":"capture-worker","pid":self.process.pid,"argv":argv})

    def enqueue(self,name,curated=False):
        self.sequence+=1
        filename=f"{datetime.datetime.now():%H%M%S}_{name}_{self.sequence:04}.png"
        path=Path("shots" if curated else "live")/filename
        self.channel.send(json.dumps({
            "name":name,"curated":curated,"path":str(path),
            "requested":time.monotonic()}).encode())
        return str(path)

    def allow_loop(self,accepted,now_ns):
        if accepted or self.last_loop_ns is None or now_ns>=self.last_loop_ns+10_000_000_000:
            self.last_loop_ns=now_ns
            return True
        return False

    def close(self):
        errors=[]
        if self.process.poll() is None:
            pending=self.sequence-self.records.read_bytes().count(b"\n")
            self.channel.send(b'{"stop":true}')
            try:
                self.process.wait(timeout=20*(pending+1)+5)
            except subprocess.TimeoutExpired:
                errors.append("capture worker exceeded bounded drain deadline")
                self.owned.stop(self.process)
        self.channel.close()
        raw=self.records.read_text()
        boundary=raw.rfind("\n")+1
        records=[json.loads(line) for line in raw[:boundary].splitlines()]
        errors.extend(row["error"] for row in records if row.get("error"))
        if self.process.returncode!=0:
            errors.append(f"capture worker exit code {self.process.returncode}")
        if len(records)!=self.sequence or raw[boundary:]:
            errors.append(f"capture completions {len(records)}/{self.sequence}; "
                          f"unfinished suffix={raw[boundary:]!r}")
        for row in records:
            if row.get("curated") and not row.get("error"):
                self.result["screenshots"].append({
                    "path":row["path"],"meaning":row["name"]})
        self.result["async_capture"]={
            "requested":self.sequence,"records":records,"errors":errors,
            "worker_returncode":self.process.poll()}
        self.result["errors"].extend("capture "+error for error in errors)
        self.log.close()


def worker(fd,root,display):
    channel=socket.socket(socket.AF_UNIX,socket.SOCK_SEQPACKET,fileno=fd)
    with (root/"capture-completions.jsonl").open("a",buffering=1) as records:
        while True:
            packet=channel.recv(8192)
            if not packet:
                break
            job=json.loads(packet)
            if job.get("stop"):
                break
            path=root/job["path"]
            temporary=path.with_name("."+path.stem+".capturing.png")
            process=None
            record=dict(job)
            try:
                process=subprocess.Popen(
                    ["import","-display",display,"-window","root",
                     "-resize","1280x>",str(temporary)],
                    stdout=subprocess.PIPE,stderr=subprocess.PIPE)
                _,stderr=process.communicate(timeout=15)
                if process.returncode:
                    raise RuntimeError(stderr.decode())
                os.replace(temporary,path)
                if job["curated"]:
                    shutil.copyfile(path,root/"live"/path.name)
                latest=root/"live"/f".latest-{os.getpid()}.png"
                shutil.copyfile(path,latest)
                os.replace(latest,root/"live/latest.png")
                record["queue_to_complete_s"]=time.monotonic()-job["requested"]
                with (root/"live/progress.log").open("a") as progress:
                    progress.write(f"{datetime.datetime.now():%H:%M:%S} | capture/done | "
                        f"비차단 촬영 완료: {job['name']} | {path.name}\n")
            except Exception as error:
                record["error"]=f"{job['name']}: {type(error).__name__}: {error}"
                if process is not None and process.poll() is None:
                    process.kill()
                    process.communicate(timeout=5)
            finally:
                temporary.unlink(missing_ok=True)
                records.write(json.dumps(record)+"\n")
    channel.close()


if __name__=="__main__":
    worker(int(sys.argv[2]),Path(sys.argv[3]),sys.argv[4])
