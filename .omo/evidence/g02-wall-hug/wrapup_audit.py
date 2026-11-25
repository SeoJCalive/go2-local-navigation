"""Read-only G02 closeout audit; no ROS, replay, GUI, motion, or cleanup starts."""
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import yaml

ROOT=Path(__file__).resolve().parent
PROJECT=ROOT.parents[2]
BASELINE="4c2b94cb2e1aad563079944f9b2ea9bfec23e43a"
run=json.loads((ROOT/"result.json").read_text())
result={"attempt":run["attempt"],"baseline":BASELINE,"checks":{}}
checks=result["checks"]
def digest(raw):
    return hashlib.sha256(raw).hexdigest()
def git(*args):
    return subprocess.check_output(["git",*args],cwd=PROJECT)

for name in ("src/go2_nav2/config/slam_mapping.yaml",
             "src/go2_mujoco_lidar/CMakeLists.txt",
             "src/go2_mujoco_lidar/launch/go2_sim_navigation.launch.py"):
    current=(PROJECT/name).read_bytes()
    assert current==git("show",BASELINE+":"+name),name
    checks[name]={"same_as_f05":True,"sha256":digest(current)}
source=PROJECT/"src/go2_nav2/config/slam_mapping.yaml"
installed=PROJECT/"install/go2_nav2/share/go2_nav2/config/slam_mapping.yaml"
assert source.read_bytes()==installed.read_bytes()
parameters=yaml.safe_load(installed.read_text())["slam_toolbox"]["ros__parameters"]
assert parameters["loop_match_maximum_variance_coarse"]==3.
checks["shared_slam_config"]={"sigma":3.,"internal_variance_m2":9.,
                             "source_equals_install":True}
installed_launch=PROJECT/"install/go2_mujoco_lidar/share/go2_mujoco_lidar/launch/go2_sim_navigation.launch.py"
assert installed_launch.read_bytes()==(PROJECT/"src/go2_mujoco_lidar/launch/go2_sim_navigation.launch.py").read_bytes()
ctest=(PROJECT/"build/go2_mujoco_lidar/CTestTestfile.cmake").read_text()
assert "slam_loop_gate" not in ctest
checks["experimental_test_registration_removed"]=True
package=subprocess.run(["dpkg","-V","ros-humble-slam-toolbox"],capture_output=True,text=True)
assert package.returncode==0 and not package.stdout and not package.stderr,package
checks["system_slam_package_unmodified"]=True
libraries=[str(p.relative_to(PROJECT)) for p in (PROJECT/"install").rglob("*")
           if p.is_file() and (p.name.startswith("libkarto") or p.name.startswith("libslam_toolbox"))]
assert not libraries,libraries
checks["shared_install_has_no_private_slam_library"]=True
sim=Path.home()/".local/share/go2-local-navigation-sim/unitree_mujoco/simulate/config.yaml"
assert digest(sim.read_bytes())==run["original_config_sha256"]
checks["original_simulator_config_preserved"]=True
assert run["residue"]==[] and run["preexisting_unchanged"]

owned_pids={p["pid"] for p in run["pids"]}
owned_pids.update(p["pid"] for p in run.get("shutdowns",[]) if "pid" in p)
running=[]
for process in Path("/proc").iterdir():
    if not process.name.isdigit() or int(process.name)==os.getpid():
        continue
    try:
        args=(process/"cmdline").read_bytes().decode().split("\0")
    except (FileNotFoundError,PermissionError,ProcessLookupError):
        continue
    if any(a.startswith(str(ROOT)+"/") or a.startswith(".omo/evidence/g02-wall-hug/")
           for a in args):
        running.append({"pid":int(process.name),"argv":args[:4]})
    if int(process.name) in owned_pids:
        running.append({"pid":int(process.name),"recorded_owned_pid":True,"argv":args[:4]})
assert not running,running
checks["owned_processes"]=running
owned_tmp=[];similar_old=[]
for path in Path("/tmp").iterdir():
    try:
        stat=path.stat()
        if stat.st_uid!=os.getuid():
            continue
        if path.name.startswith("g02-"):
            owned_tmp.append(str(path))
        if path.is_file() and stat.st_size<65536:
            text=path.read_text(errors="ignore")
            if str(ROOT) in text:
                owned_tmp.append(str(path))
            if "g02" in path.name and str(ROOT) not in text:
                similar_old.append({"path":str(path),"mtime":stat.st_mtime,
                                    "attributed_to_g02":False})
    except (OSError,PermissionError):
        continue
assert not owned_tmp,owned_tmp
checks["owned_tmp"]=owned_tmp
checks["similar_unattributed_tmp_preserved"]=similar_old
display=run["display"].split(":")[1].split(".")[0]
lock=Path("/tmp")/(".X"+display+"-lock")
if lock.exists():
    pid=int(lock.read_text().strip())
    assert pid not in owned_pids
    checks["display_lock"]={"path":str(lock),"pid":pid,"owned":False}
else:
    checks["display_lock"]={"path":str(lock),"exists":False}

documents=[
    PROJECT/"docs/stabilization-2026-11/G02_WALL_HUG.md",
    PROJECT/"src/go2_nav2/SIM_SESSION.md",
    PROJECT.parent/"go2_nav2_mujoco_bridge/README.md",
    PROJECT.parent/"go2_nav2_mujoco_bridge/XBOX_MANUAL_SLAM_RUNBOOK.txt"]
for path in documents:
    assert "TODO" not in path.read_text(),path
broken=[]
for path in documents[:2]:
    for target in re.findall(r"\]\(([^)]+)\)",path.read_text()):
        if "://" in target or target.startswith("#"):
            continue
        target=target.split("#",1)[0]
        if not (path.parent/target).exists():
            broken.append((str(path),target))
assert not broken,broken
checks["document_todos_and_links_clean"]=True
result["passed"]=True
(ROOT/"wrapup-audit.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
print("G02_WRAPUP_AUDIT_OK "+json.dumps(result,ensure_ascii=False),flush=True)
