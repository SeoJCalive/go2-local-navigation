"""목적: simulation 전용 설정과 built runtime 거리 gate의 결정성을 검사한다.

흐름: 실물 YAML hash 보존·sim 경계 → 실제 350 scan 0/1 m 및 두 반복 비교.
입출력: 고정 AGX dataset과 별도 runtime/replay engine → pytest 결과.
주의: native 자산이 없는 CI만 명시 skip한다. 로봇·GUI·ROS launch는 시작하지 않는다.
"""
import ast
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parents[2]
sys.path.insert(0, str(ROOT / "wall_hug"))
import counterfactual_replay as cf
from runtime_equivalence import compare_files


def test_simulation_gate_preserves_real_mapping_configuration():
    path = PROJECT / "src/go2_nav2/config/slam_mapping.yaml"
    assert hashlib.sha256(path.read_bytes()).hexdigest() == (
        "adc6456037cc5448d7f4a8237026b57c7a9b7f895e7e8341de68f5d7143b1529")
    launch = (PROJECT / "src/go2_mujoco_lidar/launch/go2_sim_navigation.launch.py").read_text()
    assert '"loop_closure_max_odom_distance": 1.0' in launch
    assert '"correlation_search_space_smear_deviation": .03' in launch
    assert 'get_package_prefix("slam_toolbox")' in launch


def test_built_runtime_gate_matches_replay_and_is_deterministic(tmp_path):
    runtime = PROJECT / ".omo/evidence/loop-gate-runtime/adapter/engine_replay"
    original = PROJECT / ".omo/evidence/loop-gate-replay/engine-motion-v2/build/engine_replay"
    if not runtime.is_file() or not original.is_file():
        pytest.skip("built runtime/replay engines absent; see third_party/slam_toolbox/README.md")
    manifest = json.loads(cf.MANIFEST.read_text())
    try:
        cf.checked_inputs(PROJECT, manifest)
    except cf.replay.DatasetAbsent as error:
        pytest.skip(str(error))
    environment = dict(os.environ, ROS_DOMAIN_ID="198", ROS_LOCALHOST_ONLY="1",
                       OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
    assert not environment.get("LD_PRELOAD")
    for distance in (0., 1.):
        prefixes = []
        for label, engine in (("original", original), ("runtime-a", runtime), ("runtime-b", runtime)):
            prefix = tmp_path / f"{label}-{distance:g}"
            prefixes.append(prefix)
            command = [str(engine), str(PROJECT / manifest["datasets"][0]["data"]),
                       str(PROJECT / manifest["parameters"]), str(prefix),
                       str(distance), "-1", "0", "350"]
            with prefix.with_suffix(".log").open("w") as log:
                subprocess.run(command, env=environment, stdout=log, stderr=subprocess.STDOUT,
                               check=True, timeout=90)
        compare_files(prefixes[1], prefixes[0])
        compare_files(prefixes[2], prefixes[1])


def test_gui_verifier_selects_native_argv0_and_owned_session():
    tree = ast.parse((ROOT / "wall_hug/runtime_gui.py").read_text())
    injected = next(node.value for node in ast.walk(tree)
                    if isinstance(node, ast.Constant) and isinstance(node.value, str)
                    and node.value.startswith("    native ="))
    selection = injected.split("    libraries =", 1)[0]
    rows = {
        11: dict(session=10, argv=["python3", "tf2_exec.py", "/sdk/async_slam_toolbox_node"]),
        12: dict(session=10, argv=["/sdk/async_slam_toolbox_node"]),
        13: dict(session=20, argv=["/sdk/async_slam_toolbox_node"]),
    }
    scope = dict(Path=Path, g=dict(snapshot=lambda: rows, launch=SimpleNamespace(pid=10)))
    exec("def select():\n" + selection + "    return native\n", scope)
    assert scope["select"]() == [12]
    rows[14] = dict(session=10, argv=["/sdk/async_slam_toolbox_node"])
    with pytest.raises(RuntimeError, match="expected one runtime"):
        scope["select"]()


def test_gui_template_uses_calibrated_gates_after_confirmed_route():
    if not (PROJECT / ".omo/evidence/g02-wall-hug/gui_run.py").is_file():
        pytest.skip("preserved AGX GUI helper absent; no simulator is started")
    path = ROOT / "wall_hug/runtime_gui.py"
    body = []
    for statement in ast.parse(path.read_text()).body:
        if (isinstance(statement, ast.Assign)
                and any(isinstance(target, ast.Name) and target.id == "route"
                        for target in statement.targets)):
            break
        body.append(statement)
    scope = {"__file__": str(path)}
    exec(compile(ast.Module(body=body, type_ignores=[]), str(path), "exec"), scope)
    compile(scope["source"], "preserved_gui_template", "exec")
    assert 'metrics["complete_route"]=bool(g["RESULT"]["driver"]["finished"])' in scope["source"]
    assert 'map_criteria(metrics, complete_route=metrics["complete_route"])' in scope["source"]
    assert 'expected["correlation_search_space_smear_deviation"]=.03' in scope["source"]
