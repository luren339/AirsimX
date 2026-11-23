"""Measure how fast simulation time advances relative to the wall clock.

With LockStep disabled the clock tracks real time (ratio near 1.0). With
LockStep enabled each simulator frame waits for PX4's actuator batch, so the
ratio drops; the exact number depends on the network and host load. Use this
to characterise a machine before choosing a mode, and to check that lockstep
is actually pacing the clock.

Optionally reads the UE log to report lockstep activation and reset lines.
"""

import argparse
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'PythonClient'))

import airsim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=41452)
    parser.add_argument('--vehicle', default='PX4SITL')
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--ue-log', type=Path, default=None)
    parser.add_argument('--output', type=Path, default=None)
    args = parser.parse_args()

    client = airsim.MultirotorClient(port=args.port, timeout_value=30)
    report = {'scope': 'pacing measurement; simulation only', 'wall_seconds': args.seconds}
    try:
        # Warm up so the first sample is not taken during startup transients.
        client.getMultirotorState(args.vehicle)
        start_sim = client.getMultirotorState(args.vehicle).timestamp
        start_wall = time.monotonic()
        time.sleep(args.seconds)
        end_sim = client.getMultirotorState(args.vehicle).timestamp
        end_wall = time.monotonic()
    finally:
        client.close()

    sim_seconds = (end_sim - start_sim) / 1e9
    wall_seconds = end_wall - start_wall
    ratio = sim_seconds / wall_seconds if wall_seconds else 0.0
    report.update(sim_seconds=round(sim_seconds, 3),
                  measured_wall_seconds=round(wall_seconds, 3),
                  ratio=round(ratio, 4),
                  sim_timestamps_monotonic=end_sim > start_sim)
    print('sim %.3fs / wall %.3fs = %.3fx real time'
          % (sim_seconds, wall_seconds, ratio), flush=True)

    if args.ue_log is not None:
        text = args.ue_log.read_text(encoding='utf-8', errors='replace')
        report['lockstep_activations'] = text.count('Enabling lockstep mode')
        report['lockstep_resets'] = text.count('timeout on HilActuatorControlsMessage')
        print('lockstep activations %d, resets %d'
              % (report['lockstep_activations'], report['lockstep_resets']), flush=True)

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')
        print('wrote %s' % args.output, flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
