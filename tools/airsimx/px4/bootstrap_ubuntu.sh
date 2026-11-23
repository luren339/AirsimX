#!/usr/bin/env bash
set -euo pipefail

if [[ "${WSL_DISTRO_NAME:-}" != "AirsimX-PX4-Ubuntu24" || "$(id -u)" != 0 ]]; then
    printf '%s\n' 'Run only as root in the dedicated AirsimX-PX4-Ubuntu24 distribution.' >&2
    exit 2
fi

python3 -I - <<'PY'
from pathlib import Path
import shutil
values = dict(line.split('=', 1) for line in Path('/etc/os-release').read_text().splitlines() if '=' in line)
assert values.get('ID') == 'ubuntu' and values.get('VERSION_ID', '').strip('"') == '24.04'
provenance = Path('/opt/airsimx/provenance')
provenance.mkdir(parents=True, exist_ok=True)
source = Path('/etc/apt/sources.list.d/ubuntu.sources')
backup = provenance / 'ubuntu.sources.original'
if not backup.exists():
    shutil.copy2(source, backup)
text = source.read_text()
for host in ('http://archive.ubuntu.com/ubuntu/', 'http://security.ubuntu.com/ubuntu/',
             'http://archive.ubuntu.com/ubuntu', 'http://security.ubuntu.com/ubuntu'):
    text = text.replace(host, 'https://mirrors.ustc.edu.cn/ubuntu/')
assert 'Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg' in text
source.write_text(text)
PY

export DEBIAN_FRONTEND=noninteractive
apt-get -o Acquire::Retries=3 update
apt-get -y --no-install-recommends install \
    astyle build-essential ccache cmake cppcheck file g++ gcc gdb git lcov \
    libssl-dev libxml2-dev libxml2-utils libeigen3-dev make ninja-build \
    python3 python3-dev python3-pip python3-venv python3-setuptools python3-wheel \
    rsync shellcheck unzip zip pkg-config curl ca-certificates

if ! id airsimx >/dev/null 2>&1; then
    useradd --create-home --shell /bin/bash airsimx
fi
install -d -o airsimx -g airsimx /opt/airsimx/src /opt/airsimx/venvs /opt/airsimx/runs /opt/airsimx/logs
printf '%s\n' 'Simulation-only development environment. No hardware or serial connections.' > /opt/airsimx/SITL_ONLY

dpkg-query -W > /opt/airsimx/provenance/packages-after-bootstrap.txt
python3 --version
gcc --version
cmake --version
ninja --version
printf '%s\n' 'PX4 Ubuntu build prerequisites ready.'
