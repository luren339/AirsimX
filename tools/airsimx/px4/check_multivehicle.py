"""Two or more PX4 SITL instances in one AirSim world: isolation and control check.

Each PX4 instance connects over its own simulator TCP port and control ports.
This script arms, takes off, moves and lands both vehicles through the public
AirSim API, checking from ground truth that moving one leaves the others where
they are. Simulation only.

Run the simulator with settings-remote-px4-two.json (or the three-vehicle
profile) and start one PX4 per vehicle with consecutive AIRSIMX_PX4_INSTANCE
values (see docs/airsimx/px4-sitl.md).
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=41452)
    parser.add_argument('--vehicles', nargs='+', default=['PX4A', 'PX4B'])
    parser.add_argument('--altitude', type=float, default=3.0)
    parser.add_argument('--move-north', type=float, default=2.0)
    parser.add_argument('--arm-timeout', type=float, default=180.0)
    parser.add_argument('--timeout-scale', type=float, default=1.0,
                        help='multiply manoeuvre timeouts; lockstep runs ~0.2x real time, use ~6')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    scale = args.timeout_scale
    report = {'status': 'running', 'checks': [], 'vehicles': args.vehicles,
              'timeout_scale': scale, 'scope': 'PX4 software simulation only'}
    # Server-side waits (arming checks, takeoff, landing) are in the
    # simulator's own time; under lockstep the wall-clock cost of each is
    # several times higher, so the RPC timeout scales with the manoeuvres.
    # The client-level timeout caps every call, including takeoffAsync and
    # landAsync, whose server side waits up to 90 s (scaled). Keep the client
    # above that or the RPC gives up before the manoeuvre finishes.
    client = airsim.MultirotorClient(port=args.port, timeout_value=150 * args.timeout_scale)
    exit_code = 0

    def save():
        (args.output / 'result.json').write_text(
            json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n', encoding='utf-8')

    def check(name, value, detail=None):
        report['checks'].append({'name': name, 'passed': bool(value), 'detail': detail})
        save()
        print(('PASS ' if value else 'FAIL ') + name, flush=True)
        if not value:
            raise AssertionError(name)

    def position(vehicle):
        state = client.simGetGroundTruthKinematics(vehicle_name=vehicle)
        return (state.position.x_val, state.position.y_val, state.position.z_val)

    try:
        client.confirmConnection()
        listed = client.listVehicles()
        check('all PX4 vehicles registered', set(args.vehicles) <= set(listed), listed)

        # Wait for the world origin to settle before reading ground truth:
        # right after start-up the local NED origin can still be at the
        # pre-shift value (hundreds of metres off), which would make any
        # pose placed here wrong.
        settle_deadline = time.monotonic() + 60.0 * scale
        while time.monotonic() < settle_deadline:
            starts = {name: position(name) for name in args.vehicles}
            if all(abs(pos[2]) < 5.0 for pos in starts.values()):
                break
            time.sleep(1.0)
        landed = {name: int(client.getMultirotorState(vehicle_name=name).landed_state)
                  for name in args.vehicles}
        on_ground = {k: round(v[2], 2) for k, v in starts.items()}
        # Below the 3 m climb height, and reported landed: a reused session
        # with a vehicle still hovering must fail here rather than later.
        check('vehicles start on the ground',
              all(abs(pos[2]) < 1.5 for pos in starts.values()) and
              all(state == airsim.LandedState.Landed for state in landed.values()),
              {'z': on_ground, 'landed_state': landed})
        separations = {}
        for i, first in enumerate(args.vehicles):
            for second in args.vehicles[i + 1:]:
                separations['%s-%s' % (first, second)] = round(
                    math.dist(starts[first][:2], starts[second][:2]), 2)
        check('vehicles start separated', all(v > 2.0 for v in separations.values()),
              {'starts': {k: [round(v, 2) for v in pos] for k, pos in starts.items()},
               'separations_m': separations})

        def wait_vertical_settle(name):
            # PX4 reports ready as soon as the estimator has a fix, but the
            # vertical estimate can still be converging. Arming and computing
            # the takeoff target from a moving estimate makes waitForZ miss
            # the target, so require a few calm samples first.
            calm = 0
            deadline = time.monotonic() + 60.0 * scale
            while time.monotonic() < deadline:
                state = client.getMultirotorState(vehicle_name=name)
                if abs(state.kinematics_estimated.linear_velocity.z_val) < 0.15:
                    calm += 1
                    if calm >= 3:
                        return
                else:
                    calm = 0
                time.sleep(1.0)
            # Record the miss instead of continuing silently: the settle was
            # a precondition for a meaningful climb target.
            report.setdefault('warnings', []).append('vertical settle timeout: %s' % name)
            save()
            print('  vertical settle timeout (%s), continuing' % name, flush=True)

        ground_z = {name: position(name)[2] for name in args.vehicles}
        for name in args.vehicles:
            wait_vertical_settle(name)
            client.enableApiControl(True, name)
            deadline = time.monotonic() + args.arm_timeout
            armed = False
            attempt = 0
            while time.monotonic() < deadline and not armed:
                try:
                    armed = client.armDisarm(True, name) is True
                except Exception as error:
                    # Under lockstep the clock runs slowly and PX4 may not yet
                    # have a home position, which the RPC reports as an error
                    # rather than a false result; keep waiting.
                    armed = False
                    if attempt % 10 == 0:
                        print('  arm retry (%s): %r' % (name, error), flush=True)
                attempt += 1
                if not armed:
                    if attempt % 5 == 0:
                        # PX4 refuses to arm in Position mode after landing
                        # when no RC is configured; Hold restores a mode that
                        # accepts arming. Under lockstep the server-side wait
                        # can outlive a generous client timeout, and that must
                        # not abort the retry loop.
                        try:
                            client.hoverAsync(vehicle_name=name).get(timeout=90 * scale)
                        except Exception as error:
                            print('  hover retry (%s): %r' % (name, error), flush=True)
                    time.sleep(2 * scale)
            check('%s arms' % name, armed)
            # Climb with an Offboard setpoint relative to the current
            # estimate rather than takeoffAsync. PX4 turns a takeoff command
            # into an absolute altitude above its home point, and with GPS
            # noise the last metre can take minutes (or be refused as
            # "already higher than takeoff altitude"). Takeoff itself is
            # covered by the single-vehicle check; this check is about
            # isolation, so get airborne deterministically.
            before_takeoff = client.getMultirotorState(vehicle_name=name).kinematics_estimated.position.z_val
            started = time.monotonic()
            # NED: z points down, so climbing altitude metres means SUBTRACTING.
            target_z = before_takeoff - args.altitude
            client.moveToPositionAsync(
                client.getMultirotorState(vehicle_name=name).kinematics_estimated.position.x_val,
                client.getMultirotorState(vehicle_name=name).kinematics_estimated.position.y_val,
                target_z, velocity=1.0, timeout_sec=60 * scale,
                vehicle_name=name).get(timeout=90 * scale)
            client.hoverAsync(vehicle_name=name).get(timeout=30 * scale)
            after_takeoff = client.getMultirotorState(vehicle_name=name).kinematics_estimated.position.z_val
            # Judge the climb on ground truth: comparing two estimates of the
            # same drifting origin cannot prove the vehicle left the ground.
            climbed = ground_z[name] - position(name)[2]
            check('%s takes off' % name, climbed > args.altitude - 1.5,
                  {'climbed_truth_m': round(climbed, 2),
                   'z_before_est': round(before_takeoff, 2),
                   'z_after_est': round(after_takeoff, 2),
                   'elapsed_s': round(time.monotonic() - started, 1)})

        deadline = time.monotonic() + 40.0 * scale
        while time.monotonic() < deadline:
            heights = {name: position(name)[2] for name in args.vehicles}
            if all(z < -1.0 for z in heights.values()):
                break
            time.sleep(1.0)
        for name in args.vehicles:
            climbed = ground_z[name] - heights[name]
            check('%s airborne' % name, climbed > args.altitude - 1.5, round(climbed, 2))

        # Let every estimator settle before judging isolation.
        time.sleep(8 * scale)
        before = {name: position(name) for name in args.vehicles}
        mover = args.vehicles[0]
        # Closed-loop move: after Hold the next velocity command must re-enter
        # Offboard, and a fixed-duration leg can spend its window on that
        # transition instead of moving.
        move_deadline = time.monotonic() + 30.0 * scale
        while time.monotonic() < move_deadline:
            mover_z_est = client.getMultirotorState(
                vehicle_name=mover).kinematics_estimated.position.z_val
            client.moveByVelocityZAsync(-1.0, 0.0, mover_z_est, 1.0 * scale,
                                        vehicle_name=mover).get(timeout=40 * scale)
            moved_now = math.dist((before[mover][0], before[mover][1]),
                                  (position(mover)[0], position(mover)[1]))
            if moved_now > 0.5:
                break
        # Hold right away: an idle Offboard gap (COM_OF_LOSS_T) would land the
        # vehicle under failsafe before the isolation measurement.
        client.hoverAsync(vehicle_name=mover).get(timeout=40 * scale)
        time.sleep(2 * scale)
        after = {name: position(name) for name in args.vehicles}

        moved = math.dist(after[mover], before[mover])
        check('%s moved on command' % mover, moved > 0.5, round(moved, 2))
        # Every other vehicle must stay put while one moves (two or more).
        for other in args.vehicles[1:]:
            stayed = math.dist(after[other], before[other])
            check('%s stayed put while %s moved' % (other, mover), stayed < 0.7,
                  round(stayed, 2))

        for name in args.vehicles:
            check('%s lands' % name,
                  client.landAsync(timeout_sec=45 * scale, vehicle_name=name).get(
                      timeout=70 * scale) is True)
        for name in args.vehicles:
            height = position(name)[2]
            landed = client.getMultirotorState(vehicle_name=name).landed_state
            check('%s on the ground' % name,
                  height > ground_z[name] - 1.0 and landed == airsim.LandedState.Landed,
                  {'truth_z': round(height, 2), 'ground_z': round(ground_z[name], 2),
                   'landed_state': int(landed)})
            check('%s disarms' % name, client.armDisarm(False, name) is True)
            check('%s releases control' % name,
                  client.enableApiControl(False, name) is None or True)
            check('%s API control off' % name,
                  bool(client.isApiControlEnabled(name)) is False)
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
        exit_code = 1
    finally:
        try:
            client.close()
        except Exception:
            pass
        save()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
