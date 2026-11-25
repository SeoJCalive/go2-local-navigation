"""G02 behavioral regressions: numerical controls and OS signals, no robot or GUI.

Inputs are independent synthetic poses/contacts/maps; pytest reports failures.
See wall_hug/README.md and G02_WALL_HUG.md for thresholds and provenance.
These checks establish harness contracts; physical validity is unconfirmed.
"""
from pathlib import Path
import sys
import pytest
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/"wall_hug"))
import json
import os
import select
import struct
import tempfile
import time
from PIL import Image
from async_capture import AsyncCapture
import socket
sys.path.insert(0,str(ROOT/"acceptance"))
from owned_processes import OwnedProcesses
HARNESS=ROOT/"wall_hug"


def test_full_capture_channel_reports_backpressure_without_counting_unsent():
    capture=AsyncCapture.__new__(AsyncCapture)
    capture.sequence=0
    capture.channel,peer=socket.socketpair(socket.AF_UNIX,socket.SOCK_SEQPACKET)
    capture.channel.setblocking(False)
    capture.channel.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,4096)
    try:
        for _ in range(1000):
            count=capture.sequence
            try:
                capture.enqueue("queued")
            except BlockingIOError:
                assert capture.sequence == count
                break
        else:
            pytest.fail("fixture failed to fill bounded socket")
        assert count>0
        capture.channel.shutdown(socket.SHUT_WR)
        packets=[]
        peer.settimeout(2)
        while True:
            packet=peer.recv(8192)
            if not packet:
                break
            packets.append(json.loads(packet))
        assert len(packets)==count
    finally:
        capture.channel.close(); peer.close()


def test_driver_finishes_while_capture_is_blocked():
    saved=dict(os.environ)
    owned=OwnedProcesses(lambda:None)
    capture=None
    fds=[]
    summary={}
    with tempfile.TemporaryDirectory(prefix="g02-async-unit-") as directory:
        root=Path(directory)
        for name in ("live","shots","bin"):
            (root/name).mkdir()
        for name in ("gate","notify"):
            os.mkfifo(root/name)
        gate=os.open(root/"gate",os.O_RDWR|os.O_NONBLOCK)
        notify=os.open(root/"notify",os.O_RDWR|os.O_NONBLOCK)
        fds.extend((gate,notify))
        Image.new("RGB",(4,3),(12,34,56)).save(root/"fixture.png")
        renderer=root/"bin/import"
        renderer.write_text(
            "#!/usr/bin/python3\nimport os,shutil,sys\n"
            "with open(os.environ['CAPTURE_TEST_NOTIFY'],'wb',buffering=0) as f:f.write(b'R')\n"
            "with open(os.environ['CAPTURE_TEST_GATE'],'rb',buffering=0) as f:f.read(1)\n"
            "shutil.copyfile(os.environ['CAPTURE_TEST_PNG'],sys.argv[-1])\n")
        renderer.chmod(0o755)
        os.environ.update(PATH=str(root/"bin")+":"+os.environ["PATH"],
            CAPTURE_TEST_NOTIFY=str(root/"notify"),CAPTURE_TEST_GATE=str(root/"gate"),
            CAPTURE_TEST_PNG=str(root/"fixture.png"),
            OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1")
        result={"screenshots":[],"errors":[]}
        try:
            capture=AsyncCapture(root,":fixture",owned,result)
            first=capture.enqueue("first",True)
            assert select.select([notify],[],[],10)[0],"capture renderer did not start"
            os.read(notify,1)
            began=time.monotonic()
            second=capture.enqueue("second",True)
            enqueue_elapsed=time.monotonic()-began
            assert not (root/first).exists(), "renderer must remain gated"
            assert not (root/first).exists() and not (root/second).exists()
            # Accepted events always pass; rejected events need ten seconds.
            decisions=[capture.allow_loop(accepted,stamp) for accepted,stamp in (
                (False,0),(False,9_999_999_999),(False,10_000_000_000),
                (True,10_000_000_001),(True,10_000_000_002),
                (False,20_000_000_001),(False,20_000_000_002))]
            assert decisions==[True,False,True,True,True,False,True]

            memory=os.memfd_create("g02-fake-mjData")
            os.ftruncate(memory,256)
            os.pwrite(memory,struct.pack("Q",64),0)
            os.pwrite(memory,struct.pack("Q",128),72)
            os.pwrite(memory,struct.pack("d",0),80)
            os.pwrite(memory,struct.pack("7d",0,0,.33,1,0,0,0),128)
            readonly=os.open(f"/proc/self/fd/{memory}",os.O_RDONLY)
            read_pad,write_pad=os.pipe()
            os.set_blocking(read_pad,False)
            fds.extend((memory,readonly,read_pad,write_pad))
            route=root/"route.json"
            route.write_text(json.dumps({"waypoints":[
                {"xy":[x,0],"kind":"wall","wall":"fixture"} for x in (0,.1,.2)]}))
            spec={"mem_fd":readonly,"pad_fd":write_pad,"d_address":0,
                "qpos_offset":8,"time_offset":16,"route_path":str(route),
                "output_directory":str(root)}
            (root/"driver-spec.json").write_text(json.dumps(spec))
            with (root/"driver.log").open("w") as log:
                driver=owned.start([sys.executable,str(HARNESS/"pad_driver.py"),
                    str(root/"driver-spec.json")],pass_fds=(readonly,write_pad),stdout=log,stderr=log)
                os.close(write_pad)
                fds.remove(write_pad)
                pending=b""
                packets=[]
                steps=0
                deadline=time.monotonic()+10
                while True:
                    remaining=deadline-time.monotonic()
                    assert remaining>0,"driver did not finish fixture"
                    assert select.select([read_pad],[],[],remaining)[0]
                    data=os.read(read_pad,4096)
                    if not data:
                        break
                    pending+=data
                    while len(pending)>=8:
                        packet=struct.unpack("IhBB",pending[:8])
                        pending=pending[8:]
                        packets.append(packet)
                        if packet[2:]==(2,3):
                            steps+=1
                            os.pwrite(memory,struct.pack("7d",steps*.01,0,.33,1,0,0,0),128)
                            os.pwrite(memory,struct.pack("d",steps*.020),80)
                assert driver.wait(timeout=5)==0,(root/"driver.log").read_text()
            assert [(v,k,n) for _,v,k,n in packets[:3]]==[
                (round(-.3/4*32767),2,1),(0,2,0),(0,2,3)]
            driver_result=json.loads((root/"driver-result.json").read_text())
            assert driver_result["finished"] and not driver_result["errors"]
            assert driver_result["period_s"]==.020
            trace_rows=[json.loads(line) for line in (root/"driver-trace.jsonl").read_text().splitlines()]
            assert driver_result["max_driver_loop_gap_wall_s"] == max(
                r["driver_loop_gap_wall_s"] for r in trace_rows)
            assert driver_result["max_driver_loop_gap_sim_s"] == max(
                r["driver_loop_gap_sim_s"] for r in trace_rows)
            ticks=[r["driver_wall_monotonic_s"] for r in trace_rows]
            assert all(abs((b-a)-r["driver_loop_gap_wall_s"]) < 1e-9
                for a,b,r in zip(ticks,ticks[1:],trace_rows[1:]))
            assert [(v,k,n) for _,v,k,n in packets[-20:]] == (
                [(0,2,k) for k in range(8)]+[(0,1,k) for k in range(12)])
            assert not (root/first).exists(),"capture gate was not blocking"
            # The entire driver finished while both screenshot requests were pending.
            os.write(gate,b"GG")
            capture.close()
            assert not result["errors"],result
            assert len(result["screenshots"])==2
            assert [r["name"] for r in result["async_capture"]["records"]] == ["first","second"]
            assert Image.open(root/"live/latest.png").getpixel((0,0))==(12,34,56)
            summary={"fixture":"gated PNG copier and synthetic mjData; no ROS or simulator",
                "enqueue_while_capture_blocked_s":enqueue_elapsed,
                "driver_completed_before_capture":True,"loop_capture_decisions":decisions,
                "driver_period_s":driver_result["period_s"],
                "driver_max_gap_wall_s":driver_result["max_driver_loop_gap_wall_s"],
                "driver_max_gap_sim_s":driver_result["max_driver_loop_gap_sim_s"],
                "screenshots":len(result["screenshots"])}
            capture=None
        finally:
            os.write(gate,b"GG")
            if capture is not None:
                capture.close()
            for process in reversed(owned.processes):
                owned.stop(process)
            assert owned.remaining()==[]
            for fd in fds:
                os.close(fd)
            os.environ.clear()
            os.environ.update(saved)
