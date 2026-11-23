"""Read-only diagnostic snapshot from the dedicated PX4 SITL instance."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--distro', default='AirsimX-PX4-Ubuntu24')
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    base = '/opt/airsimx/src/PX4-Autopilot-v1.17.0/build/px4_sitl_airsim/bin/'
    commands = {
        'version': ['ver', 'all'],
        'commander': ['commander', 'status'],
        'mavlink': ['mavlink', 'status'],
        'simulator': ['simulator_mavlink', 'status'],
        'vehicle_status': ['listener', 'vehicle_status', '-n', '1'],
        'local_position': ['listener', 'vehicle_local_position', '-n', '1'],
        'land_detected': ['listener', 'vehicle_land_detected', '-n', '1'],
        'actuator_armed': ['listener', 'actuator_armed', '-n', '1'],
        'manual_input_parameter': ['param', 'show', '-q', 'COM_RC_IN_MODE'],
        'offboard_loss_parameter': ['param', 'show', '-q', 'COM_OBL_RC_ACT'],
        'offboard_timeout_parameter': ['param', 'show', '-q', 'COM_OF_LOSS_T'],
    }
    report = {'timestamp_utc': datetime.now(timezone.utc).isoformat(), 'read_only': True, 'commands': {}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    for name, command in commands.items():
        argv = ['wsl.exe', '-d', args.distro, '-u', 'airsimx', '--exec', base + 'px4-' + command[0], *command[1:]]
        result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=20)
        report['commands'][name] = {'exit_code': result.returncode, 'stdout': result.stdout.decode('utf-8', errors='replace'),
                                    'stderr': result.stderr.decode('utf-8', errors='replace')}
        args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
    print(args.output)
    return 0 if all(row['exit_code'] == 0 for row in report['commands'].values()) else 1


if __name__ == '__main__':
    sys.exit(main())
