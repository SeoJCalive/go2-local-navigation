#!/usr/bin/env bash
# 목적: 고정 slam_toolbox 2.6.10과 simulation 거리 gate를 전용 overlay에 설치한다.
# 흐름: pin 확인 → opt-in 관측/게이트 patch → colcon build → library receipt.
# 입력: 선택 작업 root, 선택 원본 git 저장소. 출력: root/{build,install,log}.
# 주의: ROS Humble source 후 실행한다. /opt/ros와 실물 설정은 수정하지 않는다.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project="$(cd -- "$here/../.." && pwd)"
root="${1:-$here}"
source_repo="${2:-https://github.com/SteveMacenski/slam_toolbox.git}"
pin=8293d21fe5c816d0405e0cc1f3eb46c70659c267
source_dir="$root/build/source"
[[ "$root" = /* ]]
mkdir -p "$root/build"
if [[ ! -d "$source_dir/.git" ]]; then
  git clone --no-checkout "$source_repo" "$source_dir"
  git -C "$source_dir" checkout --detach "$pin"
  git -C "$source_dir" apply "$project/src/go2_mujoco_lidar/test/wall_hug/karto-observation.patch"
  git -C "$source_dir" apply --unidiff-zero "$here/runtime-gate.patch"
fi
[[ "$(git -C "$source_dir" rev-parse HEAD)" = "$pin" ]]
git -C "$source_dir" apply --reverse --check --unidiff-zero "$here/runtime-gate.patch"
df -h "$root"
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon --log-base "$root/log" build \
  --base-paths "$source_dir" --build-base "$root/build/workspace" \
  --install-base "$root/install" --parallel-workers 1 --allow-overriding slam_toolbox \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  --event-handlers console_direct+
python3 - "$root/install/slam_toolbox" "$pin" <<'PY'
import ctypes
import hashlib
import json
from pathlib import Path
import sys
from xml.etree import ElementTree
prefix = Path(sys.argv[1])
assert ElementTree.parse(prefix / "share/slam_toolbox/package.xml").findtext("version") == "2.6.10"
library = prefix / "lib/libkartoSlamToolbox.so"
assert ctypes.CDLL(str(library)).go2RuntimeGateVersion() == 1
record = {"commit": sys.argv[2], "version": "2.6.10",
          "karto_sha256": hashlib.sha256(library.read_bytes()).hexdigest(),
          "default_max_odom_distance_m": 0.0}
(prefix.parent / "go2-slam-runtime.json").write_text(json.dumps(record, indent=2) + "\n")
print("SLAM_RUNTIME_READY", json.dumps(record))
PY
