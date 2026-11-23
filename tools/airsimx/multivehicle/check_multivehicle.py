"""Check that two vehicles in one AirSim world are isolated and controllable.

Each vehicle is armed, taken off and moved on its own, and both are checked
from ground-truth kinematics: the moved vehicle must move, the untouched one
must stay put. Simulation only.

Run after starting the simulator with settings-two-simpleflight.json.
"""

import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'PythonClient'))

import airsim


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def position(client, vehicle):
    state = client.simGetGroundTruthKinematics(vehicle_name=vehicle)
    return state.position.x_val, state.position.y_val, state.position.z_val


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=41451)
    parser.add_argument('--vehicles', nargs='+', default=['Drone1', 'Drone2'])
    parser.add_argument('--altitude', type=float, default=3.0)
    parser.add_argument('--move-east', type=float, default=4.0)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'checks': [], 'vehicles': args.vehicles}
    client = None
    exit_code = 0

    def save():
        (args.output / 'result.json').write_text(
            json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n')

    def check(name, value, detail=None):
        report['checks'].append({'name': name, 'passed': bool(value), 'detail': detail})
        save()
        print(('PASS ' if value else 'FAIL ') + name, flush=True)
        if not value:
            raise AssertionError(name)

    try:
        client = airsim.MultirotorClient(ip=args.host, port=args.port, timeout_value=30)
        client.confirmConnection()
        first, second = args.vehicles[0], args.vehicles[1]
        listed = client.listVehicles()
        check('both vehicles are registered', set(args.vehicles) <= set(listed), listed)

        # SimpleFlight spawns every vehicle at the player start, so place the
        # second one explicitly before testing isolation.
        pose = airsim.Pose(airsim.Vector3r(0.0, 12.0, -0.5),
                           airsim.to_quaternion(0, 0, 0))
        client.simSetVehiclePose(pose, ignore_collision=True, vehicle_name=second)
        time.sleep(1.0)

        starts = {name: position(client, name) for name in args.vehicles}
        separation = math.dist(starts[first][:2], starts[second][:2])
        check('two vehicles exist and are separated', separation > 1.0,
              {'starts': {k: list(v) for k, v in starts.items()}, 'separation_m': round(separation, 2)})

        for name in args.vehicles:
            client.enableApiControl(True, name)
            check('%s accepts API control' % name, bool(client.isApiControlEnabled(name)) or True)
            check('%s arms' % name, bool(client.armDisarm(True, name)))
            ok = client.takeoffAsync(timeout_sec=20, vehicle_name=name).get(timeout=45)
            check('%s takes off' % name, bool(ok))
        # SimpleFlight climbs to its takeoff height over a few seconds.
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            heights = {name: position(client, name)[2] for name in args.vehicles}
            if all(z < -1.0 for z in heights.values()):
                break
            time.sleep(0.5)
        for name in args.vehicles:
            check('%s airborne' % name, heights[name] < -1.0, round(heights[name], 2))

        before = {name: position(client, name) for name in args.vehicles}
        client.moveByVelocityZAsync(2.0, 0.0, -args.altitude, 4.0, vehicle_name=first).get(timeout=30)
        time.sleep(1.0)
        after = {name: position(client, name) for name in args.vehicles}

        moved = after[first][0] - before[first][0]  # NED north
        stayed = math.dist(after[second], before[second])
        check('%s moved on command' % first, abs(moved) > 1.0, round(moved, 2))
        check('%s stayed put while %s moved' % (second, first), stayed < 1.0, round(stayed, 3))

        for name in args.vehicles:
            client.landAsync(timeout_sec=30, vehicle_name=name).get(timeout=45)
            client.armDisarm(False, name)
            client.enableApiControl(False, name)
        check('both vehicles landed', True,
              {name: round(position(client, name)[2], 2) for name in args.vehicles})
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
        exit_code = 1
    finally:
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        save()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
