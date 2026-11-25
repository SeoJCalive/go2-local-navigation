"""F05/G02 GUI evidence using the established G01 FIFO/process/truth helpers.

G01 is a standalone script. Load only its imports, definitions and constants,
before its top-level ROS initialization; never execute its old trial. All new
artifacts and owned processes belong to this directory. The finalizer always
reaps every owned POSIX session and checks the original config hash.
"""
import ast
import csv
import io
import json
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import time
import traceback
import xml.etree.ElementTree as ET
import re

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[2]
OLD = ROOT.parent / "g01-owner-gui"
tree = ast.parse((OLD / "owner_gui.py").read_text())
body = []
for statement in tree.body:
    if (isinstance(statement, ast.Expr) and isinstance(statement.value, ast.Call)
            and isinstance(statement.value.func, ast.Attribute)
            and isinstance(statement.value.func.value, ast.Name)
            and statement.value.func.value.id == "rclpy"
            and statement.value.func.attr == "init"):
        break
    body.append(statement)
g = {"__file__": str(OLD / "owner_gui.py"), "__name__": "g01_helpers"}
exec(compile(ast.Module(body=body, type_ignores=[]), str(OLD / "owner_gui.py"), "exec"), g)
g["ROOT"], g["FIFO"] = ROOT, ROOT / "owner-pad.fifo"
g["RESULT"] = {"goals": [], "steps": {}, "shutdowns": [], "errors": [], "screenshots": []}
g["RESULT"]["gui_script"]=str(Path(__file__).resolve())


def save_result():
    """Publish complete JSON snapshots for the independent live observers."""
    temporary=ROOT/f".result-{os.getpid()}.json"
    temporary.write_text(json.dumps(g["RESULT"],ensure_ascii=False,indent=2)+"\n")
    os.replace(temporary,ROOT/"result.json")


g["save"]=save_result
from sensor_msgs.msg import JointState
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from PIL import Image
import numpy as np

for name in ("offsets", "config-mapping.yaml", "config-nav.yaml"):
    shutil.copy2(OLD / name, ROOT / name)
for name in ("config-mapping.yaml", "config-nav.yaml"):
    path = ROOT / name
    path.write_text(path.read_text().replace(str(OLD), str(ROOT)))
(ROOT / "live").mkdir(exist_ok=True)
(ROOT / "shots").mkdir(exist_ok=True)
rclpy = g["rclpy"]
rclpy.init(signal_handler_options=g["SignalHandlerOptions"].NO)
g["node"] = node = rclpy.create_node("f05_g02_gui_observer")
node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
g["buffer"] = g["Buffer"]()
g["listener"] = g["TransformListener"](g["buffer"], node)
g["owned"] = g["OwnedProcesses"](g["pump"])
g["publisher"] = node.create_publisher(g["PoseStamped"], "/goal_pose", 10)
latest_joints = None
maps = []
capture_service=None
render_probe=None
pad_process=None


def receive(name, msg):
    if name == "sport":
        t = msg.stamp.sec + msg.stamp.nanosec / 1e9
    else:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
    g["samples"][name].append(t)


def joints(msg):
    global latest_joints
    latest_joints = msg


subscriptions = [
    node.create_subscription(g["SportModeState"], "/sportmodestate",
                             lambda m: receive("sport", m), g["qos_profile_sensor_data"]),
    node.create_subscription(g["PointCloud2"], "/utlidar/cloud",
                             lambda m: receive("cloud", m), g["qos_profile_sensor_data"]),
    node.create_subscription(g["Odometry"], "/odom",
                             lambda m: receive("odom", m), g["qos_profile_sensor_data"]),
    node.create_subscription(g["GoalStatusArray"], "/navigate_to_pose/_action/status",
                             g["status_cb"], 10),
    node.create_subscription(JointState, "/joint_states", joints, 10),
    node.create_subscription(OccupancyGrid, "/map", maps.append, QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)),
]


def rviz(name):
    p = g["start"](name, ["rviz2", "-d",
        str(PROJECT / "src/go2_mujoco_lidar/config/simulation.rviz"),
        "--ros-args", "-p", "use_sim_time:=true"],
        env_extra={"LD_LIBRARY_PATH": str(PROJECT / "third_party/tf2/install/tf2/lib")
                   + ":" + os.environ.get("LD_LIBRARY_PATH", "")})
    return p


def log_text(name):
    path = ROOT / (name + ".log")
    return path.read_text() if path.exists() else ""


def rendered():
    global render_probe
    if capture_service is None:
        shot=g["capture"]("render-check")
    else:
        if render_probe is None:
            render_probe=g["capture"]("render-check")
        if not (ROOT/render_probe).exists():
            return False
        shot,render_probe=render_probe,None
    pixels = np.asarray(Image.open(ROOT / shot))
    canvas = pixels[40:-30, 900:-20, :3]
    return int(((canvas.min(axis=2) > 195) &
                (canvas.max(axis=2)-canvas.min(axis=2) < 25)).sum()) > 500


def csv_snapshot(path):
    """Only newline-committed records belong to an actively written snapshot."""
    raw=Path(path).read_text()
    boundary=raw.rfind("\n")+1
    return np.genfromtxt(io.StringIO(raw[:boundary]),delimiter=",",names=True),raw[boundary:]


def check_model():
    expected = [f"{leg}_{joint}_joint"
                for leg in ("FR", "FL", "RR", "RL") for joint in ("hip", "thigh", "calf")]
    g["wait"](lambda: latest_joints is not None and
        all(g["buffer"].can_transform("base", f"{leg}_foot", rclpy.time.Time())
            for leg in ("FR", "FL", "RR", "RL")), 20, "joint states and leg TF")
    assert latest_joints.name == expected
    # qpos hinges follow MJCF body traversal, while LowState uses FR/FL/RR/RL.
    # Derive the independent qpos name ordering from the physics model.
    ptr = g["struct"].unpack("Q", os.pread(g["mem_fd"], 8, g["d_address"]))[0]
    qp = g["struct"].unpack("Q", os.pread(g["mem_fd"], 8, ptr + g["qpos_offset"]))[0]
    truth_q = g["struct"].unpack("12d", os.pread(g["mem_fd"], 96, qp + 56))
    physics_names = [joint.get("name") for joint in ET.parse(
        g["SIMROOT"].parent / "unitree_robots/go2/go2.xml").findall(
            "./worldbody//joint")]
    by_name = dict(zip(physics_names, truth_q))
    return {"joint_names": latest_joints.name,
            "max_joint_error_moving_sample_rad": max(abs(a-b) for a,b in zip(
                [by_name[name] for name in expected], latest_joints.position)), "four_feet_tf": True,
            "joint_publisher_count": node.count_publishers("/joint_states")}


def stop_localization(label):
    from go2_nav2.sim_stop import stop
    started = time.monotonic()
    states = stop(g["launch"].pid)
    g["stop"](g["launch"])
    g["RESULT"].setdefault("single_stop", []).append(
        {"label": label, "states": states, "wall_sec": time.monotonic()-started})


class LoopEvents:
    """Parse same-version Karto debug means and the real listener callbacks."""
    def __init__(self):
        self.best=None
        self.pending=None
        self.events=[]

    def feed(self,line):
        number=r"[-+]?(?:\d*\.\d+|\d+\.?)(?:[eE][-+]?\d+)?"
        for marker in ("BEST POSE =","bestPose:"):
            if marker in line:
                values=re.findall(number,line.split(marker,1)[1])
                if len(values)>=3:
                    self.best=[float(v) for v in values[:3]]
        match=re.search(r"G02_LOOP event=(\w+) scan=(\d+) time=([\d.]+) info=(.*)",line)
        if not match:
            return []
        kind,scan,stamp,info=match.groups()
        emitted=[]
        def finish(accepted,reason):
            if self.pending:
                self.pending.update(accepted=accepted,reason=reason)
                self.events.append(self.pending)
                emitted.append(self.pending)
                self.pending=None
        if "COARSE RESPONSE:" in info:
            finish(False,"coarse threshold")
            self.pending={"scan_id":int(scan),"time":float(stamp),
                          "candidate_map_pose":self.best,"coarse":info}
        elif "FINE RESPONSE:" in info and self.pending:
            self.pending.update(candidate_map_pose=self.best,fine=info)
        elif "REJECTED!" in info:
            finish(False,"fine threshold")
        elif kind=="accepted":
            finish(True,"actual EndLoopClosure callback")
        elif kind=="scan_end":
            finish(False,"coarse threshold")
        return emitted


def f05():
    g["prepare"]()
    g["milestone"]("F05-start", "전용 GUI와 실시간 캡처 시작", "f05-start", True)
    rows = []
    for k in range(6):
        order = "rviz-first" if k % 2 == 0 else "map-first"
        r = rviz(f"f05-rviz-{k}") if order == "rviz-first" else None
        g["new_sim"](f"f05-{k}", "switch")
        truth = g["truth"]()
        transform = json.loads((OLD / "result.json").read_text())["saved_map_world_transform"]
        yaw, tx, ty = transform
        initial = [math.cos(yaw)*truth["x"]-math.sin(yaw)*truth["y"]+tx,
                   math.sin(yaw)*truth["x"]+math.cos(yaw)*truth["y"]+ty,
                   truth["yaw"]+yaw]
        g["launch_mode"]("localization", f"f05-localization-{k}", initial)
        if r is None:
            r = rviz(f"f05-rviz-{k}")
        g["wait"](lambda: "Trying to create a map" in log_text(f"f05-rviz-{k}")
            or r.poll() is not None, 40, "RViz rendered durable map")
        if r.poll() is not None:
            raise RuntimeError(log_text(f"f05-rviz-{k}"))
        model = check_model()
        g["arrange"]()
        g["wait"](rendered, 30, "RViz rendered pixels after window resize")
        g["milestone"](f"F05-{k}", f"지도 시작순서 {k+1}/6 {order}, 관절 TF 표시",
                       f"f05-map-{k}", True)
        rows.append({"k": k, "order": order, "map_rendered": True, "model": model})
        stop_localization(str(k))
        g["stop"](g["rl"])
        g["stop"](g["sim"])
        # Keep RViz through one reset/new epoch to prove durable reacquisition.
        if k == 0:
            before = log_text(f"f05-rviz-{k}").count("Trying to create a map")
            g["new_sim"]("f05-retained", "switch")
            g["launch_mode"]("localization", "f05-retained-localization", initial)
            g["wait"](lambda: log_text(f"f05-rviz-{k}").count(
                "Trying to create a map") > before, 40, "retained RViz new epoch map")
            g["milestone"]("F05-retained", "기존 RViz의 새 epoch 지도 재수신 확인",
                           "f05-retained-map", True)
            rows.append({"order": "retained-rviz-new-epoch", "map_rendered": True})
            stop_localization("retained")
            g["stop"](g["rl"])
            g["stop"](g["sim"])
        g["stop"](r)
    g["RESULT"]["map_order_trials"] = rows


def g02():
    global capture_service,pad_process
    from geometry import evaluate, THRESHOLDS
    from geometry_msgs.msg import PoseWithCovarianceStamped
    previous=json.loads((ROOT/"result.json").read_text())
    if "map_order_trials" in previous:
        (ROOT/"f05-result.json").write_text(json.dumps(previous,indent=2))
    else:
        archive=ROOT/"attempts"/str(len(list((ROOT/"attempts").glob("*")))+1).zfill(2)
        archive.mkdir(parents=True)
        for pattern in ("result.json","physics*.csv","slam-pose.csv","g02-*.log","g02-*.csv",
                        "route-progress.json","mapping-trace.json","map-metrics.json","driver-trace.jsonl",
                        "driver-*.json","driver-*.log","capture-*.jsonl","capture-*.log","karto-*.jsonl"):
            for path in ROOT.glob(pattern):
                shutil.copy2(path,archive/path.name)
    g["RESULT"]["attempt"]=len(list((ROOT/"attempts").glob("*")))+1
    # The edge writer opens on the first link, which can be after the live
    # observer starts. Empty the previous epoch before either process opens it.
    for name in ("karto-scans.jsonl","karto-edges.jsonl","karto-candidates.jsonl"):
        (ROOT/name).write_text("")
    (ROOT/"thresholds.json").write_text(json.dumps(THRESHOLDS,indent=2))
    original_start = g["start"]
    def observed_start(name, argv, **kwargs):
        extra = kwargs.setdefault("env_extra", {}) or {}
        kwargs["env_extra"] = extra
        if name.endswith("-simulator"):
            extra.update({"LD_PRELOAD": str(ROOT / "physics_observer.so"),
                          "G02_PHYSICS_LOG": str(ROOT / (
                              "physics" if name=="g02-mapping-simulator" else name+"-physics"))})
        return original_start(name, argv, **kwargs)
    g["start"] = observed_start
    g["prepare"]()
    from async_capture import AsyncCapture
    capture_service=AsyncCapture(ROOT,g["display"],g["owned"],g["RESULT"])
    g["capture"]=capture_service.enqueue
    g["milestone"]("G02-start", "벽면 근접 매핑 시작; 임계값 파일 고정", "g02-start", True)
    g["new_sim"]("g02-mapping", "xbox")
    pose_file=(ROOT/"slam-pose.csv").open("w")
    pose_file.write("time,x,y,yaw\n")
    def pose(msg):
        p=msg.pose.pose
        q=p.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        pose_file.write(f"{msg.header.stamp.sec+msg.header.stamp.nanosec/1e9},"
                        f"{p.position.x},{p.position.y},{yaw}\n")
        pose_file.flush()
    pose_subscription=node.create_subscription(PoseWithCovarianceStamped,"/pose",pose,100)
    subscriptions.append(pose_subscription)
    args=["ros2","launch","go2_mujoco_lidar","go2_sim_navigation.launch.py",
          f"simulator_pid:={g['sim'].pid}","mode:=mapping","slam_debug_logging:=true"]
    g["launch"]=g["start"]("g02-mapping",args,cwd=PROJECT,env_extra={
        "LD_LIBRARY_PATH":str(ROOT/"karto-observed")+":"+os.environ["LD_LIBRARY_PATH"],
        "G02_SCAN_EVIDENCE":str(ROOT/"karto-scans.jsonl"),
        "G02_EDGE_EVIDENCE":str(ROOT/"karto-edges.jsonl"),
        "G02_CANDIDATE_EVIDENCE":str(ROOT/"karto-candidates.jsonl")})
    g["wait"](lambda: len(g["samples"]["cloud"])>10 and g["buffer"].can_transform(
        "map","base",rclpy.time.Time()),90,"mapping TF")
    g["wait"](lambda:(ROOT/"karto-scans.jsonl").exists() and
        (ROOT/"karto-scans.jsonl").stat().st_size>0,20,"actual Karto scan evidence")
    import yaml
    config=PROJECT/"install/go2_nav2/share/go2_nav2/config/slam_mapping.yaml"
    expected=yaml.safe_load(config.read_text())["slam_toolbox"]["ros__parameters"]
    observed={}
    for name in ("loop_match_maximum_variance_coarse",
                 "loop_match_minimum_response_coarse",
                 "loop_match_minimum_response_fine","loop_match_minimum_chain_size",
                 "loop_search_space_dimension","loop_search_maximum_distance"):
        label="g02-param-"+name
        query=g["start"](label,["ros2","param","get","/slam_toolbox",name],cwd=PROJECT)
        g["wait"](lambda:query.poll() is not None,15,"running mapper parameter "+name)
        if query.returncode!=0:
            raise RuntimeError("mapper parameter query failed: "+log_text(label))
        actual=float(log_text(label).strip().splitlines()[-1].split(":",1)[1])
        if actual!=float(expected[name]):
            raise RuntimeError(f"running mapper {name}={actual}; source={expected[name]}")
        observed[name]=actual
        g["stop"](query)
    (ROOT/"running-mapper-params.json").write_text(json.dumps(observed,indent=2))
    g["RESULT"]["running_mapper_parameters"]=observed
    r=rviz("g02-rviz")
    g["wait"](lambda:"Trying to create a map" in log_text("g02-rviz"),40,"mapping RViz")
    g["arrange"]()
    g["wait"](rendered,30,"mapping rendered")
    p=g["truth"]()
    m=g["map_pose"]()
    yaw=g["wrap"](m[2]-p["yaw"])
    transform=[yaw,m[0]-math.cos(yaw)*p["x"]+math.sin(yaw)*p["y"],
               m[1]-math.sin(yaw)*p["x"]-math.cos(yaw)*p["y"]]
    g["RESULT"]["saved_map_world_transform"]=transform
    loop_log=(ROOT/"g02-mapping.log").open()
    live_loops=LoopEvents()
    contacts=(ROOT/"physics-contacts.csv").open()
    g["RESULT"]["startup_nonfoot_contacts_recorded"]=sum(1 for line in contacts)-1
    contacts.seek(0,2)
    g["RESULT"]["mapping_start_truth"]=p
    g["RESULT"]["mapping_nonfoot_contacts"]=0
    g["RESULT"]["mapping_contact_episodes"]=0
    g["RESULT"]["mapping_max_contact_depth_m"]=0.
    g["RESULT"]["mapping_max_contact_duration_s"]=0.
    from contact_policy import ContactPolicy
    contact_policy=ContactPolicy()
    g["RESULT"].update(contact_policy.metrics)
    route=json.loads((ROOT/"route.json").read_text())
    waypoints=route["waypoints"]
    from path_tracker import PathTracker
    tracker=PathTracker(waypoints)
    waypoints=tracker.waypoints
    progress=-1
    i=0
    g["RESULT"]["thresholds"]=THRESHOLDS
    g["RESULT"]["route"]={k:v for k,v in route.items() if k!="waypoints"}
    g["milestone"]("G02-mapping-0","전체 벽면 경로 매핑 0%", "mapping-000",True)
    spec={key:g[key] for key in ("pad_fd","mem_fd","d_address","qpos_offset","time_offset")}
    spec.update(route_path=str(ROOT/"route.json"),output_directory=str(ROOT))
    (ROOT/"driver-spec.json").write_text(json.dumps(spec))
    (ROOT/"driver-status.json").unlink(missing_ok=True)
    (ROOT/"driver-result.json").unlink(missing_ok=True)
    driver_log=(ROOT/"driver-pad.log").open("w")
    g["logfiles"].append(driver_log)
    driver_argv=[sys.executable,str(ROOT/"pad_driver.py"),str(ROOT/"driver-spec.json")]
    pad_process=g["owned"].start(driver_argv,cwd=ROOT,stdout=driver_log,stderr=driver_log,
        env=dict(os.environ,OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1"),
        pass_fds=(g["pad_fd"],g["mem_fd"]))
    g["RESULT"].setdefault("pids",[]).append({
        "name":"g02-pad-driver","pid":pad_process.pid,"argv":driver_argv})
    original_neutral=g["neutral"]
    def stop_pad_then_neutral():
        if pad_process is not None and pad_process.poll() is None:
            g["stop"](pad_process)
        original_neutral()
    g["neutral"]=stop_pad_then_neutral
    g["wait"](lambda:(ROOT/"driver-status.json").exists() or pad_process.poll() is not None,
              15,"independent pad driver first feedback")
    # The observer reads this run's transform before the first possible loop.
    # It only reads telemetry and never shares the FIFO or driver cadence.
    g["save"]()
    g["start"]("g02-live-pose",
               [sys.executable,str(ROOT/"live_pose_watch.py")],cwd=ROOT)
    moving_logged=False
    last_graze_capture_ns=None
    while pad_process.poll() is None:
        tracking=json.loads((ROOT/"driver-status.json").read_text())
        if tracking is None:
            raise RuntimeError("pad driver failed before first qpos feedback; see driver-pad.log")
        p=tracking["truth"]
        start_truth=g["RESULT"]["mapping_start_truth"]
        if not moving_logged and (
                math.hypot(p["x"]-start_truth["x"],p["y"]-start_truth["y"])>.02 or
                abs(g["wrap"](p["yaw"]-start_truth["yaw"]))>.05):
            g["milestone"]("G02-driving","G02 주행 재시작; 실제 qpos 이동 확인",
                           "driving-restart",True)
            moving_logged=True
        i=tracking["index"]
        g["duration"](.02)
        for line in loop_log.readlines():
            for event in live_loops.feed(line):
                g["RESULT"].setdefault("loop_events_live",[]).append(
                    dict(event,sim_time_observed=p["sim_time"],truth=p))
                if capture_service.allow_loop(event["accepted"],time.monotonic_ns()):
                    g["milestone"]("G02-loop", "loop closure "+(
                        "수락" if event["accepted"] else "거부"),
                        "loop-"+str(len(g["RESULT"]["loop_events_live"])), True)
                else:
                    with (ROOT/"live/progress.log").open("a") as stream:
                        stream.write(f"{g['datetime'].datetime.now():%H:%M:%S} | G02/loop | "
                            f"loop 거부 scan={event['scan_id']}; 촬영10초 제한 | -\n")
        new_contacts=list(csv.reader(contacts.readlines()))
        violation=contact_policy.feed(new_contacts,g["truth"]()["sim_time"]-.002)
        g["RESULT"].update(contact_policy.metrics)
        if violation:
            g["neutral"]()
            g["milestone"]("G02-collision",violation,"collision",True)
            raise RuntimeError(violation)
        if new_contacts:
            now_ns=time.monotonic_ns()
            if last_graze_capture_ns is None or now_ns>=last_graze_capture_ns+10_000_000_000:
                g["milestone"]("G02-graze",f"스침 {len(new_contacts)}개 기록·계속; 누적 {g['RESULT']['mapping_nonfoot_contacts']}, 최대 깊이 {g['RESULT']['mapping_max_contact_depth_m']:.6f}m",
                               "graze",True)
                last_graze_capture_ns=now_ns
            else:
                with (ROOT/"live/progress.log").open("a") as stream:
                    stream.write(f"{g['datetime'].datetime.now():%H:%M:%S} | G02/graze | "
                        f"스침 {len(new_contacts)}개 추가·주행 계속, 원자료physics-contacts.csv | -\n")
        pct=int(tracking["s_m"]/tracking["length_m"]*100)//5*5
        if pct>progress:
            progress=pct
            (ROOT/"route-progress.json").write_text(json.dumps(
                {"percent":pct,"index":i,"truth":p,"wall":waypoints[i]},indent=2))
            g["milestone"](f"G02-mapping-{pct}",f"벽면 매핑 {pct}%, {waypoints[i]['wall']}",
                           f"mapping-{pct:03}",True)
    g["stop"](pad_process)
    driver_result=json.loads((ROOT/"driver-result.json").read_text())
    g["RESULT"]["driver"]={key:value for key,value in driver_result.items() if key!="mapping_trace"}
    if not driver_result["finished"] or driver_result["errors"]:
        raise RuntimeError("pad driver failed: "+json.dumps(driver_result["errors"]))
    g["neutral"]()
    violation=contact_policy.feed(list(csv.reader(contacts.readlines())),math.inf)
    g["RESULT"].update(contact_policy.metrics)
    if violation:
        raise RuntimeError(violation)
    g["RESULT"]["cross_track_error_m"]=driver_result["cross_track_error_m"]
    g["RESULT"]["max_driver_loop_gap_wall_s"]=driver_result["max_driver_loop_gap_wall_s"]
    g["RESULT"]["max_driver_loop_gap_sim_s"]=driver_result["max_driver_loop_gap_sim_s"]
    g["RESULT"]["mapping_trace"]=driver_result["mapping_trace"]
    (ROOT/"mapping-trace.json").write_text(json.dumps(driver_result["mapping_trace"],indent=2))
    # Preserve immutable LinkInfo relative measurements before the mapper exits.
    from slam_toolbox.srv import SerializePoseGraph
    graph_client=node.create_client(SerializePoseGraph,"/slam_toolbox/serialize_map")
    g["wait"](graph_client.service_is_ready,15,"final SLAM graph service")
    graph_base=ROOT/"maps"/f"attempt{g['RESULT']['attempt']}-final"
    graph_request=SerializePoseGraph.Request()
    graph_request.filename=str(graph_base)
    graph_future=graph_client.call_async(graph_request)
    g["wait"](graph_future.done,30,"final SLAM graph serialization")
    if graph_future.result().result!=0:
        raise RuntimeError("final SLAM graph serialization failed")
    node.destroy_client(graph_client)
    g["RESULT"]["saved_posegraph_base"]=str(graph_base.relative_to(ROOT))
    # Require two new map updates after the final route sample, not a fixed sleep.
    before=len(maps)
    g["wait"](lambda:len(maps)>=before+2,20,"final map updates")
    output=ROOT/("maps/wall-"+str(g["RESULT"]["attempt"]))
    output.parent.mkdir(exist_ok=True)
    save=g["start"]("g02-save-map",["ros2","run","go2_mujoco_lidar","sim_save_map",
        "--simulator-pid",str(g["sim"].pid),"--output",str(output)],cwd=PROJECT)
    g["wait"](lambda:save.poll() is not None,90,"map save")
    if save.returncode:
        raise RuntimeError("map save failed")
    g["RESULT"]["saved_map_yaml"]=str(output)+".yaml"
    g["stop"](save)
    final_map=g["map_pose"]()
    g["stop"](g["launch"])
    node.destroy_subscription(pose_subscription)
    pose_file.close()
    # Closing the GUI flushes the complete every-step independent truth stream.
    metrics=evaluate(g["WORLD"],str(output)+".yaml",transform,
                     ROOT/"live/final-map-ground-truth-overlay.png")
    (ROOT/"map-metrics.json").write_text(json.dumps(metrics,indent=2))
    g["RESULT"]["map_metrics"]=metrics
    g["milestone"]("G02-map-saved","벽면 경로 완주 및 지도 저장; GT overlay 생성",
                   "map-saved",True)
    t,truth_suffix=csv_snapshot(ROOT/"physics-truth.csv")
    s,pose_suffix=csv_snapshot(ROOT/"slam-pose.csv")
    metrics["csv_snapshot_uncommitted_suffix"]={"truth":truth_suffix,"pose":pose_suffix}
    sx=np.interp(s["time"],t["time"],t["x"])
    sy=np.interp(s["time"],t["time"],t["y"])
    yaw,tx,ty=transform
    expected_x=math.cos(yaw)*sx-math.sin(yaw)*sy+tx
    expected_y=math.sin(yaw)*sx+math.cos(yaw)*sy+ty
    errors=np.hypot(expected_x-s["x"],expected_y-s["y"])
    maximum=int(np.argmax(errors))
    metrics["slam_pose_error_m"]={"max":float(errors[maximum]),
        "p95":float(np.percentile(errors,95)),"time":float(s["time"][maximum]),
        "truth_xy":[float(sx[maximum]),float(sy[maximum])]}
    clear=t["clearance"][np.isfinite(t["clearance"])]
    metrics["actual_geometry_clearance_m"]={
        "min":float(clear.min()),"p05":float(np.percentile(clear,5)),
        "median":float(np.median(clear))}
    metrics["clearance_method"]="analytic 2D projected collision envelopes; conservative at rotated corners"
    g["RESULT"]["clearance_method"]=metrics["clearance_method"]
    driver_records=[json.loads(line) for line in (ROOT/"driver-trace.jsonl").read_text().splitlines()]
    command_times=np.array([record["time"] for record in driver_records])
    command_kinds=np.array([waypoints[record["index"]]["kind"] for record in driver_records])
    command_indices=np.searchsorted(command_times,t["time"],side="right")-1
    valid=(command_indices>=0)&(t["time"]<=command_times[-1])&np.isfinite(t["clearance"])
    sample_kinds=command_kinds[np.clip(command_indices,0,len(command_kinds)-1)]
    clear_by_kind={"wall":[],"gap":[]}
    for kind in clear_by_kind:
        clear_by_kind[kind]=t["clearance"][valid&(sample_kinds==kind)].tolist()
    metrics["clearance_by_route_kind_m"]={
        kind:{"min":float(min(values)),"p05":float(np.percentile(values,5)),
              "p10":float(np.percentile(values,10)),
              "median":float(np.median(values))}
        for kind,values in clear_by_kind.items() if values}
    metrics["contact_summary"]={key:g["RESULT"][key] for key in (
        "mapping_nonfoot_contacts","mapping_contact_episodes",
        "mapping_max_contact_depth_m","mapping_max_contact_duration_s",
        "mapping_max_deep_contact_duration_s")}
    metrics["cross_track_error_m"]=g["RESULT"]["cross_track_error_m"]
    g["RESULT"]["wall_follow_clearance_m"]=metrics["clearance_by_route_kind_m"].get("wall")
    log=log_text("g02-mapping")
    metrics["loop_closure_raw_events"]=[line for line in log.splitlines()
        if "loop" in line.lower() and any(
            word in line.lower() for word in ("clos","accept","reject","match"))]
    parsed_loops=LoopEvents()
    for line in log.splitlines():
        parsed_loops.feed(line)
    loop_events=parsed_loops.events
    for event in loop_events:
        stamp=event["time"]
        x=float(np.interp(stamp,t["time"],t["x"]))
        y=float(np.interp(stamp,t["time"],t["y"]))
        ex=math.cos(yaw)*x-math.sin(yaw)*y+tx
        ey=math.sin(yaw)*x+math.cos(yaw)*y+ty
        candidate=event["candidate_map_pose"]
        error=(float(math.hypot(candidate[0]-ex,candidate[1]-ey))
               if candidate is not None else None)
        event.update(candidate_xy_error_m=error,
            geometrically_correct_decision=(error<=.10)==event["accepted"]
                if error is not None else None,
            post_event_xy_error_m=float(np.interp(stamp,s["time"],errors)))
    metrics["loop_events"]=loop_events
    metrics["passed"]=(metrics["normal_profile_offset_m"]["p95"]<=.05 and
        metrics["normal_profile_offset_m"]["max"]<=.15 and
        metrics["ghost_wall_components"]==0 and metrics["doubled_wall_components"]==0 and
        metrics["normal_profile_thickness_m"]["p95"]<=.15 and
        metrics["slam_pose_error_m"]["max"]<=.10 and bool(loop_events) and
        g["RESULT"]["mapping_max_contact_depth_m"]<=.015 and
        g["RESULT"]["mapping_max_deep_contact_duration_s"]<=.050 and
        g["RESULT"]["mapping_max_contact_duration_s"]<=1.)
    (ROOT/"map-metrics.json").write_text(json.dumps(metrics,indent=2))
    if not metrics["passed"]:
        raise RuntimeError("mapping accuracy failed; see map-metrics.json")
    # Same physics session, saved map; direct FIFO owner ends before Nav2 bridge.
    # Xbox and bridge axis layouts differ, so restart into the documented switch profile.
    g["stop"](g["rl"]);g["stop"](g["sim"])
    g["new_sim"]("g02-navigation","switch")
    p=g["truth"]()
    initial=[math.cos(yaw)*p["x"]-math.sin(yaw)*p["y"]+tx,
             math.sin(yaw)*p["x"]+math.cos(yaw)*p["y"]+ty,p["yaw"]+yaw]
    args=["ros2","launch","go2_mujoco_lidar","go2_sim_navigation.launch.py",
          f"simulator_pid:={g['sim'].pid}","mode:=localization",
          "map:="+str(output)+".yaml","initial_pose:="+",".join(map(str,initial))]
    g["launch"]=g["start"]("g02-localization",args,cwd=PROJECT)
    g["wait"](lambda:"NAVIGATION_READY" in log_text("g02-localization"),120,"Nav2 ready")
    g["bridge"]=g["start"]("g02-bridge",["ros2","run","go2_nav2_mujoco_bridge",
        "twist_to_joystick","--ros-args","-p","fifo_path:="+str(g["FIFO"])])
    targets=[[-4.75,-3.15,math.pi],[-6.85,-3.15,-math.pi/2],
             [-6.5,-5.2,0.],[-3.95,-4.9,math.pi/2]]
    for k,target in enumerate(targets,1):
        g["run_goal"](k,target,transform)
        goal=g["RESULT"]["goals"][-1]
        finish=goal["final_truth"]["sim_time"]
        g["wait"](lambda:g["truth"]()["sim_time"]>finish+.3,5,"physics telemetry flush")
        nav_truth,nav_suffix=csv_snapshot(ROOT/"g02-navigation-simulator-physics-truth.csv")
        goal["physics_snapshot_uncommitted_suffix"]=nav_suffix
        values=nav_truth["clearance"][
            (nav_truth["time"]>=goal["start_truth"]["sim_time"])&
            (nav_truth["time"]<=finish)]
        values=values[np.isfinite(values)]
        goal["actual_min_collision_geometry_clearance_m"]=float(values.min())
        g["save"]()
    stop_localization("g02-final")
    g["RESULT"]["passed_goals"]=all(
        goal["status"]==4 and goal["xy_error_m"]<=.30 for goal in g["RESULT"]["goals"])
    if not g["RESULT"]["passed_goals"]:
        raise RuntimeError("near-wall Nav2 goal failed")


def interrupted(number, frame):
    raise RuntimeError(f"bounded trial interrupted: {number}")


signal.signal(signal.SIGTERM, interrupted)
signal.signal(signal.SIGINT, interrupted)
began = time.monotonic()
try:
    if "--g02" in sys.argv:
        g02()
    else:
        f05()
except Exception:
    g["RESULT"]["errors"].append(traceback.format_exc())
    print(g["RESULT"]["errors"][-1], flush=True)
    try:
        label="G02" if "--g02" in sys.argv else "F05"
        g["milestone"](label+"-failure", "시험 실패: "+g["RESULT"]["errors"][-1].splitlines()[-1],
                       "failure", True)
    except Exception as error:
        g["RESULT"]["errors"].append("failure milestone "+str(error))
    if "--g02" in sys.argv and g["launch"] and g["launch"].poll() is None:
        try:
            g["neutral"]()
            g["milestone"]("G02-partial", "실패 위치의 부분 지도 저장",
                           "partial", True)
            prefix=ROOT/("maps/partial-"+str(g["RESULT"].get("attempt",0)))
            prefix.parent.mkdir(exist_ok=True)
            if not prefix.with_suffix(".yaml").exists():
                save=g["start"]("partial-save", ["ros2","run","go2_mujoco_lidar",
                    "sim_save_map","--simulator-pid",str(g["sim"].pid),
                    "--output",str(prefix)],cwd=PROJECT)
                g["wait"](lambda:save.poll() is not None,60,"partial map save")
                g["stop"](save)
        except Exception as error:
            g["RESULT"]["errors"].append("partial evidence "+str(error))
finally:
    if pad_process is not None and pad_process.poll() is None:
        try:
            g["stop"](pad_process)
        except Exception as error:
            g["RESULT"]["errors"].append("pad cleanup "+str(error))
    if pad_process is not None:
        try:
            driver_result=json.loads((ROOT/"driver-result.json").read_text())
            g["RESULT"]["driver"]={key:value for key,value in driver_result.items() if key!="mapping_trace"}
            for key in ("max_driver_loop_gap_wall_s","max_driver_loop_gap_sim_s"):
                g["RESULT"][key]=driver_result[key]
        except Exception as error:
            g["RESULT"]["errors"].append("driver result "+str(error))
    if capture_service is not None:
        try:
            capture_service.close()
        except Exception as error:
            g["RESULT"]["errors"].append("capture cleanup "+str(error))
    for process in reversed(g["owned"].processes):
        try:
            g["stop"](process)
        except Exception as error:
            g["RESULT"]["errors"].append("cleanup " + str(error))
    for fd in (g["pad_fd"], g["mem_fd"]):
        if fd is not None:
            os.close(fd)
    g["FIFO"].unlink(missing_ok=True)
    result = g["RESULT"]
    result["residue"] = g["owned"].remaining()
    result["shutdowns"] = g["owned"].stops
    result["config_preserved"] = g["hashlib"].sha256(
        (g["SIMROOT"] / "config.yaml").read_bytes()).hexdigest() == result.get(
            "original_config_sha256")
    result["preexisting_unchanged"] = result.get("preexisting") == g["existing_stack"](PROJECT)
    result["wall_sec"] = time.monotonic()-began
    result["passed"] = (not result["errors"] and not result["residue"] and
                        result["config_preserved"] and result["preexisting_unchanged"])
    g["save"]()
    label="G02" if "--g02" in sys.argv else "F05"
    with (ROOT/"live/progress.log").open("a") as stream:
        stream.write(f"{g['datetime'].datetime.now():%H:%M:%S} | {label}/teardown | "
            f"합격={result['passed']}, 잔류={len(result['residue'])}, "
            f"원본 config 유지={result['config_preserved']}, "
            f"기존 identity 유지={result['preexisting_unchanged']} | latest.png\n")
    node.destroy_node()
    rclpy.shutdown()
    print(label+"_GUI_FINISHED " + json.dumps(
        {k:result[k] for k in ("passed", "residue", "errors")}), flush=True)
    sys.exit(0 if result["passed"] else 1)
