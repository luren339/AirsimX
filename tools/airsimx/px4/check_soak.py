"""Long-running soak check: repeated takeoff/hover/move/land cycles on PX4.

Each cycle arms, climbs to a hover height with an Offboard setpoint, holds,
commands a short leg, lands and disarms, asserting along the way from ground
truth. The cycle count or wall-clock duration is bounded (default 30 cycles
or 30 minutes, whichever comes first). Simulation only.

The soak is meant to catch slow failures that a single flight hides: estimator
drift, battery behaviour, connection leaks, growing offsets between the
flight controller and the simulator.

Setpoints are re-anchored against ground truth by default. The controller
works in its own local frame, whose origin drifts relative to ground truth
under sensor noise and dead reckoning (~0.28 m per cycle was measured); a
soak that commands fixed local coordinates therefore walks away from where it
started and eventually presses into the ground or a wall. ``anchored_setpoint``
adds the current estimator offset (estimate - truth) to each ground-truth
target, so every cycle returns to the same true position. ``--no-reanchor``
keeps the old local-frame behaviour, for comparison.

Run the simulator with a profile whose Parameters include COM_DISARM_PRFLT
and a slow battery drain (all reference profiles do), then start one PX4.
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


def anchored_setpoint(target_truth, estimate_xyz, truth_xyz, enabled=True):
    """Local-frame setpoint that should place the vehicle at target_truth.

    The controller works in its own local frame, whose origin drifts relative
    to ground truth under sensor noise and dead reckoning. Adding the current
    estimator offset (estimate - truth) to a ground-truth target compensates
    for the drift at command time. With ``enabled`` false the target is
    returned unchanged, i.e. it is interpreted as a local setpoint.
    """
    target_truth = tuple(float(v) for v in target_truth)
    if not enabled:
        return target_truth
    return tuple(target + (estimate - truth)
                 for target, estimate, truth in
                 zip(target_truth, estimate_xyz, truth_xyz))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, default=41452)
    parser.add_argument('--vehicle', default='PX4SITL')
    parser.add_argument('--cycles', type=int, default=30)
    parser.add_argument('--minutes', type=float, default=30.0)
    parser.add_argument('--altitude', type=float, default=3.0)
    parser.add_argument('--hover-seconds', type=float, default=10.0)
    parser.add_argument('--move-metres', type=float, default=2.0)
    parser.add_argument('--arm-timeout', type=float, default=120.0)
    parser.add_argument('--offboard-loss-t', type=float, default=1.0,
                        help='COM_OF_LOSS_T the profile sets; the hold cadence must stay below it')
    parser.add_argument('--drift-limit', type=float, default=2.5,
                        help='max ground-truth distance from the home position after a cycle')
    parser.add_argument('--no-reanchor', dest='reanchor', action='store_false',
                        help='command local-frame targets directly (pre-re-anchoring behaviour)')
    args = parser.parse_args()
    if args.cycles < 1 or args.minutes <= 0:
        parser.error('cycles must be >= 1 and minutes > 0; an empty run proves nothing')
    if not (args.hover_seconds > 0):
        parser.error('hover-seconds must be positive')
    if args.altitude < 1.5:
        parser.error('altitude must be >= 1.5 m so the airborne check is meaningful')
    args.output.mkdir(parents=True, exist_ok=False)
    report = {'status': 'running', 'cycles': [], 'scope': 'PX4 software simulation only',
              'reanchor': bool(args.reanchor),
              'offboard_loss_t_s': args.offboard_loss_t,
              'limits': {'cycles': args.cycles, 'minutes': args.minutes}}
    client = airsim.MultirotorClient(port=args.port, timeout_value=150)
    exit_code = 0
    started = time.monotonic()
    prev_offset = None

    def save():
        (args.output / 'result.json').write_text(
            json.dumps(json_safe(report), indent=2, allow_nan=False) + '\n', encoding='utf-8')

    def check(name, value, detail=None):
        report['checks_total'] = report.get('checks_total', 0) + 1
        report['checks_passed'] = report.get('checks_passed', 0) + (1 if value else 0)
        report['checks'].append({'name': name, 'passed': bool(value), 'detail': detail})
        if not value:
            report['first_failure'] = report.get('first_failure') or {'name': name, 'detail': detail}
            save()
            print('FAIL ' + name, flush=True)
            raise AssertionError(name)
        save()

    def truth_xyz():
        position = client.simGetGroundTruthKinematics(vehicle_name=args.vehicle).position
        return (position.x_val, position.y_val, position.z_val)

    def estimate_xyz():
        position = client.getMultirotorState(
            vehicle_name=args.vehicle).kinematics_estimated.position
        return (position.x_val, position.y_val, position.z_val)

    def fly_to_truth(target_truth, record=None, key=None, timeout_sec=60, leg=None):
        """Fly to a ground-truth target (local setpoint via re-anchoring).

        No Hold afterwards: consecutive calls stay in Offboard, so the
        position keeps being corrected as the estimator drifts. Parking in
        Hold between legs re-introduces the drift (Hold pins local
        coordinates, which move in truth terms).
        """
        estimate_now, truth_now = estimate_xyz(), truth_xyz()
        local = anchored_setpoint(target_truth, estimate_now, truth_now,
                                  enabled=args.reanchor)
        if record is not None and leg is not None:
            record.setdefault('legs', {})[leg] = {
                'target_truth': [round(v, 2) for v in target_truth],
                'offset': [round(e - t_, 2) for e, t_ in zip(estimate_now, truth_now)],
                'setpoint_local': [round(v, 2) for v in local]}
        return client.moveToPositionAsync(*local, velocity=1.0,
                                          timeout_sec=timeout_sec,
                                          vehicle_name=args.vehicle).get(
            timeout=timeout_sec + 30)

    def hold_truth(target_truth, seconds):
        """Hold a ground-truth position, sampling and re-anchoring throughout.

        Two things made the first version pass while PX4 was failsafing
        during every hover:

        - ``moveToPositionAsync`` stops sending setpoints once the vehicle is
          inside its arrival tolerance (~0.5 m), so the Offboard link goes
          quiet and PX4 raises its offboard-loss failsafe (COM_OF_LOSS_T,
          1 s in these profiles). Each iteration therefore issues a *moving*
          setpoint that is never reached, keeping the link alive.
        - Sampling only after the command returned measured the vehicle at
          the moment it had just been re-corrected, so any excursion between
          commands was invisible. Sample on a fixed cadence instead, in
          parallel with commanding.

        Returns (worst deviation in metres, number of samples, seconds held).
        """
        deadline = time.monotonic() + seconds
        worst = 0.0
        samples = 0
        held_from = time.monotonic()
        while time.monotonic() < deadline:
            # Re-issue the (re-anchored) target every ~150 ms. Once the
            # vehicle is inside moveToPosition's arrival tolerance the call
            # returns immediately without further setpoints, so the cadence —
            # not the call — is what keeps Offboard active below
            # COM_OF_LOSS_T. The target itself is never offset, so this
            # cannot drag the vehicle anywhere.
            try:
                fly_to_truth(target_truth, timeout_sec=1)
            except Exception as error:
                report.setdefault('warnings', []).append('hold command: %r' % error)
            now = truth_xyz()
            worst = max(worst, max(abs(n - tt) for n, tt in zip(now, target_truth)))
            samples += 1
            time.sleep(0.15)
        return worst, samples, round(time.monotonic() - held_from, 1)

    try:
        client.confirmConnection()
        report['checks'] = []

        # Wait for the world origin to settle before reading ground truth:
        # right after start-up the local origin can still be at its pre-shift
        # value (hundreds of metres off), and every later target would be
        # measured against the wrong home.
        settle_deadline = time.monotonic() + 60.0
        home = truth_xyz()
        landed = int(client.getMultirotorState(vehicle_name=args.vehicle).landed_state)
        while time.monotonic() < settle_deadline and not (
                abs(home[2]) < 5.0 and landed == airsim.LandedState.Landed):
            time.sleep(1.0)
            home = truth_xyz()
            landed = int(client.getMultirotorState(vehicle_name=args.vehicle).landed_state)
        check('world origin settled on a landed vehicle',
              abs(home[2]) < 5.0 and landed == airsim.LandedState.Landed,
              {'truth_home': [round(v, 2) for v in home], 'landed_state': landed})
        report['truth_home'] = [round(v, 2) for v in home]

        cycle = 0
        while cycle < args.cycles and (time.monotonic() - started) / 60.0 < args.minutes:
            cycle += 1
            record = {'cycle': cycle, 'started_s': round(time.monotonic() - started, 1)}
            report['cycles'].append(record)  # recorded even if a check fails
            client.enableApiControl(True, args.vehicle)

            deadline = time.monotonic() + args.arm_timeout
            armed = False
            attempt = 0
            while time.monotonic() < deadline and not armed:
                try:
                    armed = client.armDisarm(True, args.vehicle) is True
                except Exception:
                    armed = False
                attempt += 1
                if not armed:
                    if attempt % 6 == 0:
                        try:
                            client.hoverAsync(vehicle_name=args.vehicle).get(timeout=60)
                        except Exception:
                            pass
                    time.sleep(2)
            record['arm_attempts'] = attempt
            check('cycle %d arms' % cycle, armed, {'attempts': attempt})

            # Every cycle aims at the same ground-truth home, so drift cannot
            # accumulate: the re-anchoring term grows as the estimator origin
            # drifts, while the vehicle stays where it started.
            climb_target = (home[0], home[1], home[2] - args.altitude)
            fly_to_truth(climb_target, record, leg='climb')
            up = home[2] - truth_xyz()[2]
            check('cycle %d airborne' % cycle, up > args.altitude - 1.5, round(up, 2))

            estimate_now, truth_now = estimate_xyz(), truth_xyz()
            offset = tuple(e - t for e, t in zip(estimate_now, truth_now))
            record['estimator_offset_m'] = [round(v, 2) for v in offset]
            if prev_offset is not None:
                jump = math.dist(offset, prev_offset)
                check('cycle %d estimator offset stable' % cycle, jump < 3.0,
                      {'jump_m': round(jump, 2), 'offset': record['estimator_offset_m']})
            prev_offset = offset

            worst, samples, held = hold_truth(climb_target, args.hover_seconds)
            hovered = home[2] - truth_xyz()[2]
            record['hover'] = {'worst_deviation_m': round(worst, 2), 'samples': samples,
                               'held_s': held}
            check('cycle %d hover holds altitude' % cycle,
                  samples >= 5 and held >= 0.8 * args.hover_seconds and
                  worst < 2.0 and abs(hovered - args.altitude) < 2.0,
                  record['hover'] | {'final_height_m': round(hovered, 2)})

            before = truth_xyz()
            out_target = (home[0] + args.move_metres, home[1], home[2] - args.altitude)
            fly_to_truth(out_target, record, leg='out')
            after_out = truth_xyz()
            moved = math.dist(before[:2], after_out[:2])
            check('cycle %d moved on command' % cycle, moved > 0.5, round(moved, 2))
            fly_to_truth(climb_target, record, leg='back')
            # Hold immediately after the return leg: this profile lands the
            # vehicle when Offboard is lost for COM_OF_LOSS_T (1 s), and an
            # idle gap after a motion command counts as Offboard loss.
            client.hoverAsync(vehicle_name=args.vehicle).get(timeout=60)

            back = truth_xyz()
            returned = math.dist(before[:2], back[:2])
            record['truth_after_return'] = [round(back[0], 2), round(back[1], 2)]
            record['correction_m'] = round(math.hypot(*record['estimator_offset_m']), 2)
            check('cycle %d returned to start' % cycle, returned < 1.5, round(returned, 2))
            drift_home = math.dist(back[:2], home[:2])
            record['drift_from_home_m'] = round(drift_home, 2)
            check('cycle %d stays near its home position' % cycle,
                  drift_home < args.drift_limit, round(drift_home, 2))

            check('cycle %d lands' % cycle,
                  client.landAsync(timeout_sec=60, vehicle_name=args.vehicle).get(timeout=90) is True)
            landed_z = truth_xyz()[2]
            delta = landed_z - home[2]
            check('cycle %d on the ground' % cycle, abs(delta) < 0.5, round(delta, 2))
            check('cycle %d disarms' % cycle,
                  client.armDisarm(False, args.vehicle) is True)
            client.enableApiControl(False, args.vehicle)
            record['landing_truth_delta_m'] = round(delta, 2)
            record['finished_s'] = round(time.monotonic() - started, 1)
            save()
            print('cycle %d ok (%.0fs, drift %.2fm, correction %.2fm)'
                  % (cycle, record['finished_s'], drift_home, record['correction_m']),
                  flush=True)

        check('soak ran at least one cycle', len(report['cycles']) >= 1,
              {'cycles': len(report['cycles'])})
        check('every cycle completed', all('finished_s' in c for c in report['cycles']),
              {'cycles': len(report['cycles'])})
        report['status'] = 'passed'
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
        exit_code = 1
    finally:
        # A failed cycle must not leave the vehicle airborne in Offboard with
        # API control still enabled: land, disarm and release, and record what
        # state was found.
        try:
            final = client.getMultirotorState(vehicle_name=args.vehicle)
            report['final_landed_state'] = int(final.landed_state)
            if int(final.landed_state) != int(airsim.LandedState.Landed):
                client.hoverAsync(vehicle_name=args.vehicle).get(timeout=30)
                client.landAsync(timeout_sec=60, vehicle_name=args.vehicle).get(timeout=90)
            client.armDisarm(False, args.vehicle)
            client.enableApiControl(False, args.vehicle)
        except Exception as error:
            report['cleanup_error'] = repr(error)
        try:
            client.close()
        except Exception:
            pass
        report['elapsed_minutes'] = round((time.monotonic() - started) / 60.0, 1)
        report['cycles_completed'] = len(report['cycles'])
        save()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
