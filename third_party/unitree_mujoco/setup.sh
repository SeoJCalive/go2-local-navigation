#!/usr/bin/env bash
# 목적: 전용 디렉터리에 고정 MuJoCo/SDK/simulator를 준비하고 Release로 빌드한다.
# 흐름: upstream 고정 checkout -> 배포물 해시 검증 -> 상태 패치 -> SDK 설치 -> 빌드.
# 입력: $1 설치 루트(필수), JOBS(기본 4). 출력 경로와 사전 apt 의존성은 README.md 참조.
# GitHub HTTPS 접근이 필요하다. 기존 공유 dependency를 대상으로 지정하지 않는다.
# GUI, DDS/ROS node, 제어 프로그램은 실행하지 않는다.
set -euo pipefail

patch_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
root=$(realpath -m -- "${1:?usage: bash setup.sh DEDICATED_INSTALL_ROOT}")
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

archive="$root/mujoco-$mujoco_version-linux-$(uname -m).tar.gz"
if [[ ! -f "$archive" ]]; then
  curl --fail --location --connect-timeout 30 --max-time 300 --output "$archive.part" \
    "https://github.com/google-deepmind/mujoco/releases/download/$mujoco_version/$(basename "$archive")"
  mv "$archive.part" "$archive"
fi
echo "$digest  $archive" | sha256sum --check -
tar -xzf "$archive" -C "$root"
if [[ ! -e "$root/unitree_mujoco/simulate/mujoco" ]]; then
  ln -s "$root/mujoco-$mujoco_version" "$root/unitree_mujoco/simulate/mujoco"
fi
[[ $(readlink -f "$root/unitree_mujoco/simulate/mujoco") == "$root/mujoco-$mujoco_version" ]]

patch="$patch_dir/0001-sport-state-simulation-time-imu.patch"
if git -C "$root/unitree_mujoco" diff --quiet -- simulate/src/unitree_sdk2_bridge.h; then
  git -C "$root/unitree_mujoco" apply --check "$patch"
  git -C "$root/unitree_mujoco" apply "$patch"
else
  git -C "$root/unitree_mujoco" apply --reverse --check "$patch"
fi
# 다른 tracked 변경이 섞인 checkout은 빌드하지 않는다.
git -C "$root/unitree_mujoco" diff --exit-code -- . ':!simulate/src/unitree_sdk2_bridge.h'
git -C "$root/unitree_mujoco" diff -- simulate/src/unitree_sdk2_bridge.h | cmp - "$patch"

cmake -S "$root/unitree_sdk2" -B "$root/unitree_sdk2/build" \
  -DBUILD_EXAMPLES=OFF -DCMAKE_INSTALL_PREFIX="$root/sdk-install" -DCMAKE_INSTALL_LIBDIR=lib
cmake --build "$root/unitree_sdk2/build" --parallel "${JOBS:-4}"
cmake --install "$root/unitree_sdk2/build"
cmake -S "$root/unitree_mujoco/simulate" -B "$root/unitree_mujoco/simulate/build" \
  -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release \
  -Dunitree_sdk2_DIR="$root/sdk-install/lib/cmake/unitree_sdk2"
cmake --build "$root/unitree_mujoco/simulate/build" --parallel "${JOBS:-4}"
