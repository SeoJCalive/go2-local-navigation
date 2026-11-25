#!/usr/bin/env bash
# Build only repo-local replay dependencies. Input is an isolated output prefix.
# The engine commit and both patches are checked before any compilation.
set -euo pipefail
here=$(cd -- "$(dirname -- "$0")" && pwd)
prefix=${1:?Usage: build.sh NEW_ISOLATED_PREFIX [SLAM_TOOLBOX_GIT_SOURCE]}
source_repo=${2:-"$here/../../../../../.omo/evidence/g02-wall-hug/karto-source"}
pin=8293d21fe5c816d0405e0cc1f3eb46c70659c267
test ! -e "$prefix/source"
mkdir -p "$prefix"
git clone --no-hardlinks --no-checkout "$source_repo" "$prefix/source"
git -C "$prefix/source" checkout --detach "$pin"
git -C "$prefix/source" apply --check "$here/../karto-observation.patch"
git -C "$prefix/source" apply "$here/../karto-observation.patch"
git -C "$prefix/source" apply --check "$here/karto-gate.patch"
git -C "$prefix/source" apply "$here/karto-gate.patch"
cmake -S "$here" -B "$prefix/build" -DENGINE_SOURCE="$prefix/source"
cmake --build "$prefix/build" --target engine_replay -j2
printf 'COUNTERFACTUAL_BUILD_OK %s\n' "$prefix/build/engine_replay"
