#!/usr/bin/env bash
# 목적: Humble 환경에서 고정 tf2만 프로젝트 전용 prefix에 설치한다.
# 입력: 선택적 절대 작업 루트. 기본값은 이 스크립트 디렉터리다.
# 출력: root/{build,install,log}, source/version/library hash receipt. 전역 ROS는 수정하지 않는다.
set -euo pipefail
root="${1:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)}"
[[ "$root" = /* ]] || { echo "absolute prefix required" >&2; exit 1; }
commit=404b7224d623d614f18fa9738dbf1716403d857e
source_dir="$root/build/tf2-source"
mkdir -p "$root/build"
if [[ ! -d "$source_dir/.git" ]]; then
  git init "$source_dir"
  git -C "$source_dir" remote add origin https://github.com/ros2/geometry2.git
  git -C "$source_dir" fetch --depth 1 origin "$commit"
  git -C "$source_dir" checkout --detach FETCH_HEAD
fi
[[ "$(git -C "$source_dir" rev-parse HEAD)" = "$commit" ]]
[[ -z "$(git -C "$source_dir" status --porcelain --untracked-files=no)" ]]
df -h "$root"
colcon --log-base "$root/log" build \
  --base-paths "$source_dir/tf2" --build-base "$root/build/tf2-pin" \
  --install-base "$root/install" \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
python3 - "$root/install" "$commit" <<'PY'
import hashlib
import json
from pathlib import Path
import sys
from xml.etree import ElementTree
prefix = Path(sys.argv[1])
assert ElementTree.parse(prefix / "tf2/share/tf2/package.xml").findtext("version") == "0.25.24"
record = {
    "commit": sys.argv[2], "version": "0.25.24",
    "sha256": hashlib.sha256((prefix / "tf2/lib/libtf2.so").read_bytes()).hexdigest(),
}
(prefix / "go2-tf2-pin.json").write_text(json.dumps(record, indent=2) + "\n")
print("TF2_PIN_READY", json.dumps(record))
PY
