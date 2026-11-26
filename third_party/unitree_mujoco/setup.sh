#!/usr/bin/env bash
# 목적: 전용 디렉터리에 고정 MuJoCo/SDK/simulator를 준비하고 Release로 빌드한다.
# 흐름: upstream 고정 checkout -> 배포물 해시 검증 -> 상태·GUI 패치 -> SDK 설치 -> 빌드.
# 입력: $1 설치 루트(기본 ~/.local/share/go2-local-navigation-sim), JOBS(기본 4).
# BUILD_ACCEPTANCE=1이면 RL_ROOT의 기존 object로 전용 검증 driver도 만든다.
# GitHub HTTPS 접근이 필요하다. 공유 갱신은 백업 후 APPLY_AGX_CONFIG=1로 명시한다.
# GUI, DDS/ROS node, 제어 프로그램은 실행하지 않는다.
set -euo pipefail

patch_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(realpath -m -- "${1:-$HOME/.local/share/go2-local-navigation-sim}")
sim_commit=1eb6642e3f3fdfb7fb13a9794fd6a2dd93ea0e7d
sdk_commit=9754cd153af3da471b0fe5f3aa535e426fb11db3
mujoco_version=3.3.6

# 재실행은 checkout을 덮어쓰지 않고 pin과 patch 적용 상태를 검사한다.
checkout() {
  local url=$1 commit=$2 dest=$3
  if [[ ! -e "$dest" ]]; then
    git init "$dest"
    git -C "$dest" remote add origin "$url"
    git -C "$dest" fetch --depth 1 origin "$commit"
    git -C "$dest" checkout --detach FETCH_HEAD
  fi
  [[ $(git -C "$dest" rev-parse HEAD) == "$commit" ]]
  [[ $(git -C "$dest" remote get-url origin) == "$url" ]]
}

case "$(uname -m)" in
  aarch64) digest=dd6389a008a1a4e4678b4a6118c0a099621f3f26721b6f05bb689a92250850b5 ;;
  x86_64) digest=049204172901afad251070385a6badf46d795ebe47403d093f8469557eeeab5a ;;
  *) echo "Unsupported Linux architecture: $(uname -m)" >&2; exit 1 ;;
esac
mkdir -p "$root"
checkout https://github.com/unitreerobotics/unitree_mujoco.git "$sim_commit" "$root/unitree_mujoco"
checkout https://github.com/unitreerobotics/unitree_sdk2.git "$sdk_commit" "$root/unitree_sdk2"
[[ -z $(git -C "$root/unitree_sdk2" status --porcelain --untracked-files=no) ]]

dist_root="${MUJOCO_DIST_ROOT:-$root}"
mkdir -p "$dist_root"
archive="$dist_root/mujoco-$mujoco_version-linux-$(uname -m).tar.gz"
if [[ ! -f "$archive" ]]; then
  curl --fail --location --connect-timeout 30 --max-time 300 --output "$archive.part" \
    "https://github.com/google-deepmind/mujoco/releases/download/$mujoco_version/$(basename "$archive")"
  mv "$archive.part" "$archive"
fi
echo "$digest  $archive" | sha256sum --check -
tar -xzf "$archive" -C "$dist_root"
if [[ ! -e "$root/unitree_mujoco/simulate/mujoco" ]]; then
  ln -s "$dist_root/mujoco-$mujoco_version" "$root/unitree_mujoco/simulate/mujoco"
fi
[[ $(readlink -f "$root/unitree_mujoco/simulate/mujoco") == "$dist_root/mujoco-$mujoco_version" ]]

patch="$patch_dir/0001-sport-state-simulation-time-imu.patch"
epoch_patch="$patch_dir/0003-gui-integer-epoch.patch"
viewer_patch="$patch_dir/0004-viewer-camera.patch"
# 0004는 0003의 main context를 확장한다. 최종 패치부터 기존 설치를 인식한다.
# 임의 변경은 아래 독립 clean apply와 byte 비교로 거부한다.
if ! git -C "$root/unitree_mujoco" apply --reverse --check "$viewer_patch" 2>/dev/null &&
   ! git -C "$root/unitree_mujoco" apply --reverse --check "$epoch_patch" 2>/dev/null; then
  if git -C "$root/unitree_mujoco" diff --quiet -- simulate/src/unitree_sdk2_bridge.h; then
    git -C "$root/unitree_mujoco" apply --check "$patch"
    git -C "$root/unitree_mujoco" apply "$patch"
  else
    git -C "$root/unitree_mujoco" apply --reverse --check "$patch"
  fi
  git -C "$root/unitree_mujoco" apply --check "$epoch_patch"
  git -C "$root/unitree_mujoco" apply "$epoch_patch"
fi
if ! git -C "$root/unitree_mujoco" apply --reverse --check "$viewer_patch" 2>/dev/null; then
  git -C "$root/unitree_mujoco" apply --check "$viewer_patch"
  git -C "$root/unitree_mujoco" apply "$viewer_patch"
fi
if [[ "${APPLY_AGX_CONFIG:-0}" == 1 ]]; then
  config_patch="$patch_dir/0002-agx-config-snapshot.patch"
  if git -C "$root/unitree_mujoco" diff --quiet -- simulate/config.yaml; then
    git -C "$root/unitree_mujoco" apply "$config_patch"
  fi
fi
# pinned upstream의 독립 임시 tree에 clean apply하고 모든 변경 파일을 byte 비교한다.
reference=$(mktemp -d)
trap 'rm -rf -- "$reference"' EXIT
git -C "$root/unitree_mujoco" archive "$sim_commit" | tar -x -C "$reference"
git -C "$reference" apply "$patch"
git -C "$reference" apply "$epoch_patch"
git -C "$reference" apply "$viewer_patch"
if [[ "${APPLY_AGX_CONFIG:-0}" == 1 ]]; then
  git -C "$reference" apply "$config_patch"
fi
for path in simulate/src/unitree_sdk2_bridge.h simulate/src/main.cc \
  simulate/src/simulation_step.h simulate/src/viewer_camera.h simulate/config.yaml; do
  cmp "$reference/$path" "$root/unitree_mujoco/$path"
done
git -C "$root/unitree_mujoco" diff --exit-code -- . \
  ':!simulate/src/unitree_sdk2_bridge.h' ':!simulate/src/main.cc' ':!simulate/config.yaml'
echo "PINNED_PATCH_BYTE_COMPARE PASS"

sdk_build="${SDK_BUILD_ROOT:-$root/unitree_sdk2/build}"
cmake -S "$root/unitree_sdk2" -B "$sdk_build" \
  -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX="$root/sdk-install" -DCMAKE_INSTALL_LIBDIR=lib
df -h "$root"
cmake --build "$sdk_build" --parallel "${JOBS:-4}"
cmake --install "$sdk_build"
cmake -S "$root/unitree_mujoco/simulate" -B "$root/unitree_mujoco/simulate/build" \
  -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release \
  -Dunitree_sdk2_DIR="$root/sdk-install/lib/cmake/unitree_sdk2"
df -h "$root"
cmake --build "$root/unitree_mujoco/simulate/build" --parallel "${JOBS:-4}"

if [[ "${BUILD_ACCEPTANCE:-0}" == 1 ]]; then
  project=$(cd "$patch_dir/../.." && pwd)
  UNITREE_MUJOCO_ROOT="$root/unitree_mujoco" MUJOCO_ROOT="$dist_root/mujoco-$mujoco_version" \
    cmake -S "$project/src/go2_mujoco_lidar/test/acceptance" -B "$root/acceptance-build" \
      -DCMAKE_BUILD_TYPE=Release \
      -DRL_ROOT="${RL_ROOT:-$(dirname "$project")/go2_rl_locomotion_baseline}" \
      -DSDK_ROOT="$root/unitree_sdk2" \
      -Dunitree_sdk2_DIR="$root/sdk-install/lib/cmake/unitree_sdk2"
  df -h "$root"
  cmake --build "$root/acceptance-build" --parallel "${JOBS:-4}"
  ctest --test-dir "$root/acceptance-build" --output-on-failure
fi
