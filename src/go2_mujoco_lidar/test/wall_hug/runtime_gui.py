"""목적: 보존 G02 GUI 경로를 built runtime overlay로 한 번 재실행한다.

흐름: 기존 orchestration의 SDK 주입만 제거하고 정식 driver/capture/contact를 연결한다.
--record-video를 명시하면 같은 DISPLAY의 root 화면을 live/*.mp4에 녹화하고 EOS로 닫는다.
입출력: 고정 route·GUI/physics assets → g02-wall-hug/live와 새 attempt 증거.
주의: ROS/SLAM overlay source 후 --g02로 실행한다. 보정된 지도 기준만 공유한다.
과거 evidence helper가 필요한 AGX 전용 runner이며 물리 로봇 실행은 지원하지 않는다.
"""
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
PROJECT = HERE.parents[3]
ROOT = PROJECT / ".omo/evidence/g02-wall-hug"


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise RuntimeError(f"G02 preserved helper contract changed: {old[:80]}")
    return source.replace(old, new, 1)


source = (ROOT / "gui_run.py").read_text()
source = replace_once(source, "capture_service=None",
                      "capture_service=None\nvideo_service=None")
source = replace_once(source, "global capture_service,pad_process",
                      "global capture_service,pad_process,video_service")
source = replace_once(
    source, '    capture_service=AsyncCapture(ROOT,g["display"],g["owned"],g["RESULT"])',
    '    capture_service=AsyncCapture(ROOT,g["display"],g["owned"],g["RESULT"])\n'
    '    if "--record-video" in sys.argv:\n'
    '        from record_video import VideoRecorder\n'
    '        video_service=VideoRecorder(ROOT,g["display"],g["owned"],g["RESULT"])')
source = replace_once(
    source, "    if capture_service is not None:",
    '    if video_service is not None:\n'
    '        try:\n'
    '            video_service.close()\n'
    '        except Exception as error:\n'
    '            g["RESULT"]["errors"].append("video cleanup "+str(error))\n'
    '    if capture_service is not None:')
source = replace_once(
    source, "벽면 근접 매핑 시작; 임계값 파일 고정",
    "1.0 m 게이트·smear .03 runtime 벽면 매핑 시작; GT 보정 기준·동일 경로")
source = replace_once(
    source, 'tree = ast.parse((OLD / "owner_gui.py").read_text())',
    'helper = (OLD / "owner_gui.py").read_text()\n'
    'helper = helper.replace(\'capture_process = start("live-capture", '
    '[sys.executable, str(ROOT/"live_capture.py"), str(ROOT)])\', '
    '\'capture_process = None\')\n'
    'tree = ast.parse(helper)')
source = replace_once(
    source, 'for name in ("karto-scans.jsonl","karto-edges.jsonl","karto-candidates.jsonl"):\n'
    '        (ROOT/name).write_text("")',
    'from pose_watch import prepare_run\n    prepare_run(ROOT)')
source = replace_once(
    source, '"LD_LIBRARY_PATH":str(ROOT/"karto-observed")+":"+os.environ["LD_LIBRARY_PATH"],',
    '"LD_LIBRARY_PATH":os.environ["LD_LIBRARY_PATH"],')
source = replace_once(
    source, 'expected=yaml.safe_load(config.read_text())["slam_toolbox"]["ros__parameters"]',
    'expected=yaml.safe_load(config.read_text())["slam_toolbox"]["ros__parameters"]\n'
    '    expected["loop_closure_max_odom_distance"]=1.0\n'
    '    expected["correlation_search_space_smear_deviation"]=.03')
source = replace_once(
    source, 'for name in ("loop_match_maximum_variance_coarse",',
    'for name in ("correlation_search_space_smear_deviation",'
    '"loop_closure_max_odom_distance","loop_match_maximum_variance_coarse",')
source = replace_once(
    source, 'str(ROOT/"pad_driver.py")', 'str(Path(__runtime_modules__)/"pad_driver.py")')
source = replace_once(
    source, '    metrics["passed"]=(metrics["normal_profile_offset_m"]["p95"]<=.05 and\n'
    '        metrics["normal_profile_offset_m"]["max"]<=.15 and\n'
    '        metrics["ghost_wall_components"]==0 and metrics["doubled_wall_components"]==0 and\n'
    '        metrics["normal_profile_thickness_m"]["p95"]<=.15 and\n',
    '    from geometry import map_criteria\n'
    '    metrics["complete_route"]=bool(g["RESULT"]["driver"]["finished"])\n'
    '    metrics["criteria"]=map_criteria(metrics, complete_route=metrics["complete_route"])\n'
    '    metrics["passed"]=(all(metrics["criteria"].values()) and\n')
source = replace_once(
    source, '    import yaml\n    config=PROJECT/',
    '''    native = [pid for pid,row in g["snapshot"]().items()
              if row["session"] == g["launch"].pid
              and Path(row["argv"][0]).name == "async_slam_toolbox_node"]
    if len(native) != 1:
        raise RuntimeError("expected one runtime SLAM native child")
    libraries = sorted({line.split()[-1] for line in
                        Path(f"/proc/{native[0]}/maps").read_text().splitlines()
                        if any(name in line for name in (
                            "libkartoSlamToolbox.so", "libtoolbox_common.so",
                            "libceres_solver_plugin.so"))})
    expected_prefix = str(PROJECT/"third_party/slam_toolbox/install/slam_toolbox/lib")+"/"
    if len(libraries) != 3 or any(not value.startswith(expected_prefix) for value in libraries):
        raise RuntimeError("mixed/system SLAM runtime libraries: "+str(libraries))
    g["RESULT"]["slam_runtime_libraries"] = libraries
    import yaml
    config=PROJECT/''')
# This request asks for the mapping route; acceptance owns subsequent Nav2 goals.
source = replace_once(
    source, '    # Same physics session, saved map; direct FIFO owner ends before Nav2 bridge.',
    '    return\n    # Same physics session, saved map; direct FIFO owner ends before Nav2 bridge.')
route = ROOT / "route.json"
if hashlib.sha256(route.read_bytes()).hexdigest() != (
        "8b9cbeb85a44c795137447e8350b46aadba1fd21288497dbb1ec12a88207246e"):
    raise RuntimeError("G02 attempt-6 route hash changed")
receipt = dict(
    original_gui_sha256=hashlib.sha256((ROOT / "gui_run.py").read_bytes()).hexdigest(),
    route_sha256=hashlib.sha256(route.read_bytes()).hexdigest(),
    runtime_overlay=str(PROJECT / "third_party/slam_toolbox/install/slam_toolbox"),
    parameter_max_odom_distance_m=1.0,
    driver=str(HERE / "pad_driver.py"), capture=str(HERE / "async_capture.py"))
(PROJECT / ".omo/evidence/loop-gate-runtime/gui-receipt.json").write_text(
    json.dumps(receipt, indent=2) + "\n")
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))
scope = {"__name__": "__main__", "__file__": str(ROOT / "gui_run.py"),
         "__runtime_modules__": str(HERE)}
exec(compile(source, str(ROOT / "gui_run.py"), "exec"), scope)
