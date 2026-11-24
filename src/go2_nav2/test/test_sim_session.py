"""파일 bytes·OS flock·독립 거리/원주 기하로 sim 시작 경계를 검증한다.

Nav2 map YAML schema와 PGM magic은 map_server 입력 계약이다. 아래 3-4-5
삼각형과 +/-pi 경계는 구현 상수 전사 대신 해석 가능한 독립 기대값이다.
AMCL lifecycle·실제 초기화 수락과 TF owner는 별도 headless runtime에서 확인한다.
"""

import hashlib
import json
import math
from pathlib import Path
import sys

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).parents[1]))
from go2_nav2.sim_session import (
    TF2_COMMIT, initial_pose, map_selection, mode_owners, pose_accepted, session_lock, tf2_library,
)


@pytest.fixture
def map_file(tmp_path):
    (tmp_path / "map.pgm").write_bytes(b"P5\n2 2\n255\n\xff\xff\x00\xcd")
    path = tmp_path / "map.yaml"
    path.write_text(
        "image: map.pgm\nresolution: 0.05\norigin: [-1, -2, 0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return path


def test_map_resolves_image_relative_to_yaml_and_hashes_bytes(map_file, monkeypatch, tmp_path):
    monkeypatch.chdir("/")
    selected = map_selection(map_file)
    assert selected["pgm"] == str(tmp_path / "map.pgm")
    assert selected["yaml_sha256"] == hashlib.sha256(map_file.read_bytes()).hexdigest()
    before = selected["pgm_sha256"]
    (tmp_path / "map.pgm").write_bytes(b"P2\n1 1\n255\n0\n")
    assert map_selection(map_file)["pgm_sha256"] != before


def test_missing_yaml_is_rejected(tmp_path):
    with pytest.raises(FileNotFoundError):
        map_selection(tmp_path / "absent.yaml")


def test_missing_image_is_rejected(map_file):
    map_file.with_suffix(".pgm").unlink()
    with pytest.raises(FileNotFoundError):
        map_selection(map_file)


@pytest.mark.parametrize("replacement", [
    {"resolution": 0}, {"resolution": float("nan")}, {"origin": [0, 0]},
    {"origin": [0, float("inf"), 0]}, {"occupied_thresh": 0.1},
    {"negate": 2}, {"image": "map.png"},
])
def test_invalid_map_metadata_is_rejected(map_file, replacement):
    document = yaml.safe_load(map_file.read_text())
    document.update(replacement)
    map_file.write_text(yaml.safe_dump(document))
    with pytest.raises((ValueError, FileNotFoundError)):
        map_selection(map_file)


def test_non_pgm_bytes_are_rejected(map_file):
    map_file.with_suffix(".pgm").write_bytes(b"not an image")
    with pytest.raises(ValueError, match="magic"):
        map_selection(map_file)


@pytest.mark.parametrize("value", ["", "1,2", "1,2,3,4", "nan,0,0", "0,inf,0"])
def test_initial_pose_is_explicit_finite_triplet(value):
    with pytest.raises(ValueError):
        initial_pose(value)


def test_initial_pose_preserves_nonzero_map_coordinates():
    assert initial_pose("1.25,-2.5,3.14") == (1.25, -2.5, 3.14)


def test_acceptance_uses_euclidean_distance_and_wrapped_yaw():
    assert pose_accepted((1, 2, math.pi-0.01), 1.3, 2.4, -math.pi+0.01)
    assert not pose_accepted((1, 2, 0), 1.31, 2.4, 0)
    assert not pose_accepted((1, 2, 0), 1, 2, math.pi/2)
    assert not pose_accepted((1, 2, 0), float("nan"), 2, 0)


def test_mapping_and_localization_have_disjoint_single_owners():
    assert mode_owners("mapping", "", "") == ("slam_toolbox",)
    assert mode_owners("localization", "/saved.yaml", "1,2,3") == ("amcl",)
    for mode, filename, pose in [
        ("mapping", "/saved.yaml", ""), ("mapping", "", "0,0,0"),
        ("localization", "", "0,0,0"), ("localization", "/saved.yaml", ""),
        ("both", "/saved.yaml", "0,0,0"),
    ]:
        with pytest.raises(ValueError):
            mode_owners(mode, filename, pose)


def test_os_lock_prevents_mode_overlap_until_previous_session_exits(tmp_path):
    first = session_lock(41, tmp_path)
    try:
        with pytest.raises(RuntimeError, match="stop it before switching"):
            session_lock(41, tmp_path)
        other_domain = session_lock(42, tmp_path)
        other_domain.close()
    finally:
        first.close()
    next_mode = session_lock(41, tmp_path)
    next_mode.close()


def test_tf2_requires_source_version_and_exact_built_bytes(tmp_path):
    library = tmp_path / "tf2/lib/libtf2.so"
    library.parent.mkdir(parents=True)
    library.write_bytes(b"independent fixture, not a loadable library")
    package = tmp_path / "tf2/share/tf2/package.xml"
    package.parent.mkdir(parents=True)
    package.write_text("<package><version>0.25.24</version></package>")
    record = {
        "commit": TF2_COMMIT, "version": "0.25.24",
        "sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
    }
    (tmp_path / "go2-tf2-pin.json").write_text(json.dumps(record))
    assert tf2_library(tmp_path) == library
    library.write_bytes(b"changed library")
    with pytest.raises(RuntimeError, match="hash mismatch"):
        tf2_library(tmp_path)
    library.write_bytes(b"independent fixture, not a loadable library")
    package.write_text("<package><version>0.25.23</version></package>")
    with pytest.raises(RuntimeError, match="version"):
        tf2_library(tmp_path)
