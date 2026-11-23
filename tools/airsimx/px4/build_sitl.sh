#!/usr/bin/env bash
# Build the pinned PX4 external-MAVLink simulator target on any Ubuntu 24.04 host.
# Usage: build_sitl.sh [bare-source-cache-or-git-url]
# Environment: AIRSIMX_PX4_ROOT overrides the install root (default /opt/airsimx).
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="${AIRSIMX_PX4_ROOT:-/opt/airsimx}"
cache_arg="${1:-}"
tag="$(python3 -I -c 'import json,sys; d=json.load(open(sys.argv[1]))["px4"]; print(d["tag"])' "$script_dir/environment.lock.json")"
commit="$(python3 -I -c 'import json,sys; print(json.load(open(sys.argv[1]))["px4"]["commit"])' "$script_dir/environment.lock.json")"
repository="$(python3 -I -c 'import json,sys; print(json.load(open(sys.argv[1]))["px4"]["repository"])' "$script_dir/environment.lock.json")"
version="${tag#v}"
source="$root/src/PX4-Autopilot-$tag"
venv="$root/venvs/px4-$tag"
logs="$root/logs"
mkdir -p "$root/src" "$root/venvs" "$logs"

if [[ "$(id -u)" == 0 ]]; then
    printf '%s\n' 'Refusing to build as root; use the normal development user.' >&2
    exit 2
fi

# Prepare a local bare cache: reuse the provided path, clone a URL, or fetch from GitHub.
local_cache="$root/src/.cache/px4-$tag.git"
if [[ -n "$cache_arg" && -d "$cache_arg" ]]; then
    if [[ ! -d "$local_cache" ]]; then
        cp -R -- "$cache_arg" "$local_cache"
        chmod -R go-rwx "$local_cache"
    fi
elif [[ ! -d "$local_cache" ]]; then
    mkdir -p "$(dirname "$local_cache")"
    url="$repository"
    [[ -n "$cache_arg" && "$cache_arg" == *"://"* ]] && url="$cache_arg"
    git -c http.version=HTTP/1.1 clone --bare --depth 1 --single-branch --branch "$tag" "$url" "$local_cache"
fi
[[ "$(git --git-dir="$local_cache" rev-parse "$tag^{commit}")" == "$commit" ]] || {
    printf '%s\n' "Source cache does not contain the pinned $tag commit $commit" >&2
    exit 1
}

if [[ ! -d "$source/.git" ]]; then
    if [[ -e "$source" ]]; then
        printf '%s\n' 'Refusing to overwrite an existing source directory.' >&2
        exit 2
    fi
    git clone --no-hardlinks --no-checkout "$local_cache" "$source"
    git -C "$source" checkout --detach "$commit"
    git -C "$source" remote set-url origin "$repository"
fi
[[ "$(git -C "$source" rev-parse HEAD)" == "$commit" ]]
git -C "$source" diff --ignore-submodules=all --exit-code
# A shallow bare cache has no submodule content. Fetch only what the external
# MAVLink SITL target actually builds; a full recursive init also pulls Gazebo,
# NuttX and vendor SDKs that this simulator-only build never uses.
if [[ ! -f "$source/src/modules/mavlink/mavlink/pymavlink/tools/mavgen.py" ||
      ! -f "$source/src/lib/events/libevents/events/common.json" ]]; then
    git -C "$source" submodule update --init \
        src/lib/events/libevents \
        src/lib/crypto/libtomcrypt \
        src/lib/crypto/libtommath
    # The mavlink repository itself carries pymavlink and message definitions
    # as nested submodules used by the header generator.
    git -C "$source" submodule update --init --recursive src/modules/mavlink/mavlink
fi

if [[ ! -x "$venv/bin/python" ]]; then
    python3 -m venv "$venv"
fi
"$venv/bin/python" -I -m pip -q install --upgrade pip
"$venv/bin/python" -I -m pip -q install -r "$source/Tools/setup/requirements.txt"
"$venv/bin/python" -I -m pip freeze > "$logs/px4-python-requirements-$tag.txt"

"$venv/bin/python" -I - "$source" <<'PY'
from pathlib import Path
import sys
source = Path(sys.argv[1])
original = (source / 'boards/px4/sitl/default.px4board').read_text()
# AirSim supplies physics and sensors over MAVLink HIL. Gazebo bridges, the
# uXRCE-DDS agent and vendor drivers are not part of that path and their
# submodules are intentionally not fetched.
disabled = ('CONFIG_MODULES_SIMULATION_GZ_MSGS', 'CONFIG_MODULES_SIMULATION_GZ_BRIDGE',
            'CONFIG_MODULES_SIMULATION_GZ_PLUGINS', 'CONFIG_MODULES_UXRCE_DDS_CLIENT')
lines = []
for line in original.splitlines():
    name = line.split('=', 1)[0].strip()
    lines.append(name + '=n' if name in disabled else line)
expected = '\n'.join(lines) + '\nCONFIG_MODULES_SIMULATION_SIMULATOR_MAVLINK=y\n'
overlay = source / 'boards/px4/sitl/airsim.px4board'
if overlay.exists() and overlay.read_text() != expected and 'CONFIG_MODULES_SIMULATION_SIMULATOR_MAVLINK=y' not in overlay.read_text():
    raise RuntimeError('Unexpected existing custom SITL configuration; not overwritten')
overlay.write_text(expected)
PY

export PATH="$venv/bin:$PATH"
export GIT_CONFIG_COUNT=3
export GIT_CONFIG_KEY_0=http.version GIT_CONFIG_VALUE_0=HTTP/1.1
export GIT_CONFIG_KEY_1=http.lowSpeedLimit GIT_CONFIG_VALUE_1=1024
export GIT_CONFIG_KEY_2=http.lowSpeedTime GIT_CONFIG_VALUE_2=60
cd "$source"
make px4_sitl_airsim -j"${AIRSIMX_BUILD_JOBS:-$(nproc)}"
git submodule status --recursive > "$logs/px4-submodules-$tag.txt"
cp boards/px4/sitl/airsim.px4board "$logs/airsim.px4board"
printf '%s\n' "PX4 $tag external-MAVLink simulator target built at $source/build/px4_sitl_airsim"
