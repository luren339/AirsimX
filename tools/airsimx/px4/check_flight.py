"""Bounded, simulation-only PX4 flight smoke check through the public AirSim API."""

import argparse
import json
import math
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'PythonClient'))
import airsim


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=41452)
    parser.add_argument('--vehicle', default='PX4SITL')
    parser.add_argument('--hover-seconds', type=float, default=5)
    parser.add_argument('--settle-seconds', type=float, default=8)
    parser.add_argument('--position', action='store_true')
    parser.add_argument('--arm-wait-seconds', type=float, default=45)
    args = parser.parse_args()
    if not 1 <= args.hover_seconds <= 60:
        parser.error('hover seconds must be between 1 and 60')
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'checks': [], 'states': [], 'scope': 'PX4 software simulation only'}
    client = airsim.MultirotorClient(port=args.port, timeout_value=45)
    armed = controlled = False

    def save():
        (args.output / 'result.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')

    def check(name, value, detail=None):
        report['checks'].append({'name': name, 'passed': bool(value), 'detail': detail})
        save()
        print(('PASS ' if value else 'FAIL ') + name, flush=True)
        if not value:
            raise AssertionError(name)

    def truth_z():
        k = client.simGetGroundTruthKinematics(args.vehicle)
        return k.position.z_val

    def state(label):
        value = client.getMultirotorState(args.vehicle)
        k = value.kinematics_estimated
        row = {'label': label, 'timestamp': int(value.timestamp), 'landed': int(value.landed_state),
               'position': [k.position.x_val, k.position.y_val, k.position.z_val],
               'velocity': [k.linear_velocity.x_val, k.linear_velocity.y_val, k.linear_velocity.z_val]}
        report['states'].append(row)
        check(label + ' finite', all(math.isfinite(x) for x in row['position'] + row['velocity']))
        return row

    try:
        check('ping', client.ping() is True)
        settings = json.loads(client.getSettingsString())
        vehicle = settings.get('Vehicles', {}).get(args.vehicle, {})
        check('software PX4 profile', vehicle.get('VehicleType') == 'PX4Multirotor' and vehicle.get('UseSerial') is False)
        check('initially landed', state('initial')['landed'] == airsim.LandedState.Landed)
        ground = report['states'][-1]['position']
        ground_truth_z = truth_z()
        client.enableApiControl(True, args.vehicle)
        controlled = True
        check('API requested', client.isApiControlEnabled(args.vehicle))
        # PX4 briefly denies arming after the first GPS fix ("height estimate not
        # stable"), and after a landing it stays in Position mode, whose arming
        # checks require manual-control input that this RC-less SITL lacks.
        # Switching to Hold (hoverAsync) restores an armable mode; retry bounded.
        armed = client.armDisarm(True, args.vehicle) is True
        if not armed:
            client.hoverAsync(vehicle_name=args.vehicle).get()
        deadline = time.monotonic() + args.arm_wait_seconds
        attempts = 0
        while time.monotonic() < deadline and not armed:
            attempts += 1
            armed = client.armDisarm(True, args.vehicle) is True
            if not armed:
                time.sleep(2)
        report['arm_attempts'] = attempts
        check('arm', armed)
        check('takeoff command', client.takeoffAsync(timeout_sec=25, vehicle_name=args.vehicle).get() is True)
        airborne = state('airborne')
        check('altitude gain', ground[2] - airborne['position'][2] > 1, airborne['position'])
        check('flying state', airborne['landed'] == airsim.LandedState.Flying)
        check('native hold command', client.hoverAsync(vehicle_name=args.vehicle).get() is True)
        # Let the estimator settle after takeoff before timing the hover: with
        # realistic GPS noise the EKF corrects its initial altitude and
        # position over the first several seconds, and that convergence is not
        # hover drift.
        settle_deadline = time.monotonic() + args.settle_seconds
        while time.monotonic() < settle_deadline:
            time.sleep(0.5)
        anchor = state('hold start')
        start = time.monotonic()
        drift = 0.0
        while time.monotonic() - start < args.hover_seconds:
            time.sleep(0.5)
            sample = state('hold')
            drift = max(drift, math.dist(anchor['position'], sample['position']))
            check('remains flying', sample['landed'] == airsim.LandedState.Flying)
            check('hold drift <= 1m', drift <= 1, drift)
        report['hover_max_drift_m'] = drift
        if args.position:
            target = [anchor['position'][0] + 2, anchor['position'][1], anchor['position'][2]]
            # 40 s: with realistic GPS noise the EKF spends the first seconds
            # of each position leg correcting, and PX4 re-enters Offboard
            # after the hover command.
            check('position command', client.moveToPositionAsync(*target, velocity=1, timeout_sec=40,
                  vehicle_name=args.vehicle).get() is True)
            reached = state('position target')
            check('position error < 0.5m', math.dist(reached['position'], target) < 0.5, reached['position'])
            check('hold after movement', client.hoverAsync(vehicle_name=args.vehicle).get() is True)
        check('land command', client.landAsync(timeout_sec=35, vehicle_name=args.vehicle).get() is True)
        landed = state('landed')
        check('landed state', landed['landed'] == airsim.LandedState.Landed)
        # Compare landing height against takeoff height on the ground truth.
        # The EKF's altitude is relative to an origin set from noisy GPS, so
        # comparing two estimates against each other measures origin drift,
        # not landing accuracy.
        landed_truth_z = truth_z()
        report['landing_truth_delta_m'] = landed_truth_z - ground_truth_z
        check('land altitude (ground truth)', abs(landed_truth_z - ground_truth_z) < 0.35,
              {'truth_delta_m': report['landing_truth_delta_m'],
               'estimate_delta_m': landed['position'][2] - ground[2]})
        check('disarm', client.armDisarm(False, args.vehicle) is True)
        armed = False
        client.enableApiControl(False, args.vehicle)
        controlled = False
        check('API released', not client.isApiControlEnabled(args.vehicle))
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
    finally:
        if armed:
            try:
                recovery = airsim.MultirotorClient(port=args.port, timeout_value=45)
                recovery.cancelLastTask(args.vehicle)
                report['cleanup_land'] = recovery.landAsync(timeout_sec=35, vehicle_name=args.vehicle).get()
                if recovery.getMultirotorState(args.vehicle).landed_state == airsim.LandedState.Landed:
                    report['cleanup_disarm'] = recovery.armDisarm(False, args.vehicle)
                    recovery.enableApiControl(False, args.vehicle)
                recovery.close()
            except Exception as error:
                report['cleanup_error'] = repr(error)
        elif controlled:
            try:
                client.enableApiControl(False, args.vehicle)
            except Exception as error:
                report['cleanup_error'] = repr(error)
        client.close()
        save()
    return 0 if report['status'] == 'passed' else 1


if __name__ == '__main__':
    sys.exit(main())
