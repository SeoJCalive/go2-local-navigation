"""시뮬레이션 지도·pose·단일 모드와 private tf2의 시작 경계를 검증한다.

ROS node 생성 전에 launch와 지도 저장 CLI가 호출한다. 지도 hash는 YAML/PGM의
현재 bytes를 식별하며 지도 정확도를 뜻하지 않는다. flock은 이 진입점의 동일
domain 세션을 직렬화한다. 기존 별도 SLAM/AMCL은 runtime guard에서도 거부한다.
"""

import fcntl
import hashlib
import json
import math
from pathlib import Path
from xml.etree import ElementTree

import yaml

TF2_COMMIT = "404b7224d623d614f18fa9738dbf1716403d857e"


def map_selection(filename):
    """Nav2 map metadata와 실제 PGM을 확인하고 두 파일 hash를 반환한다."""
    path = Path(filename).expanduser().resolve(strict=True)
    document = yaml.safe_load(path.read_text())
    image = (path.parent / document["image"]).resolve(strict=True)
    if image.suffix.lower() != ".pgm" or image.stat().st_size == 0:
        raise ValueError("map image must be a nonempty PGM")
    if image.read_bytes()[:2] not in (b"P2", b"P5"):
        raise ValueError("map image has no PGM magic")
    resolution = float(document["resolution"])
    origin = document["origin"]
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError("map resolution must be positive and finite")
    if len(origin) != 3 or not all(math.isfinite(float(v)) for v in origin):
        raise ValueError("map origin must contain finite x,y,yaw")
    free, occupied = float(document["free_thresh"]), float(document["occupied_thresh"])
    if not 0 <= free < occupied <= 1 or document["negate"] not in (0, 1):
        raise ValueError("invalid map occupancy thresholds or negate")
    return {
        "yaml": str(path), "pgm": str(image),
        "yaml_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "pgm_sha256": hashlib.sha256(image.read_bytes()).hexdigest(),
    }


def initial_pose(value):
    """필수 x,y,yaw argument를 SI 단위의 유한한 세 값으로 해석한다."""
    values = tuple(float(item) for item in value.split(","))
    if len(values) != 3 or not all(math.isfinite(item) for item in values):
        raise ValueError("initial_pose must be finite x,y,yaw in metres/radians")
    return values


def pose_accepted(expected, x, y, yaw):
    """초기화 수락 window: 거리 0.5 m, 주기 경계를 감싼 각도 0.5 rad."""
    values = (*expected, x, y, yaw)
    if not all(math.isfinite(item) for item in values):
        return False
    angle = math.atan2(math.sin(yaw-expected[2]), math.cos(yaw-expected[2]))
    return math.hypot(x-expected[0], y-expected[1]) <= 0.5 and abs(angle) <= 0.5


def mode_owners(mode, map_path, pose):
    """두 모드의 입력을 확인하며 map->odom owner를 정확히 하나 반환한다."""
    if mode == "mapping":
        if map_path or pose:
            raise ValueError("mapping does not accept a saved map or initial pose")
        return ("slam_toolbox",)
    if mode == "localization":
        if not map_path or not pose:
            raise ValueError("localization requires map and initial_pose:=x,y,yaw")
        initial_pose(pose)
        return ("amcl",)
    raise ValueError("mode must be mapping or localization")


def session_lock(domain, directory=Path("/tmp")):
    """launch가 종료될 때까지 반환한 file을 보관한다. lock file은 지우지 않는다."""
    lock = (directory / f"go2-sim-session-{domain}.lock").open("a+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError("another sim session owns this domain; stop it before switching mode")
    return lock


def tf2_library(prefix):
    """고정 source/version과 build 때 기록한 library hash를 검사한다."""
    root = Path(prefix).expanduser().resolve(strict=True)
    version = ElementTree.parse(root / "tf2/share/tf2/package.xml").findtext("version")
    record = json.loads((root / "go2-tf2-pin.json").read_text())
    library = (root / "tf2/lib/libtf2.so").resolve(strict=True)
    digest = hashlib.sha256(library.read_bytes()).hexdigest()
    if version != "0.25.24" or record != {
        "commit": TF2_COMMIT, "version": "0.25.24", "sha256": digest,
    }:
        raise RuntimeError("tf2 pin/version/library hash mismatch; run third_party/tf2/setup.sh")
    return library
