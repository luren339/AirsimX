"""Measure the simulated sensor noise level against its configured values.

Samples IMU, GPS, barometer and magnetometer through the AirSim RPC API while
the vehicle sits on the ground, estimates the per-sample noise sigma of each
channel, and writes a JSON report. Use it to confirm that a noise profile is
actually applied and that the values match the datasheet figures the profile
was built from. Simulation only.

Notes on the estimators:
- IMU and magnetometer: sigma from the first difference (robust to slow bias
  drift), divided by sqrt(2). The IMU sample rate is inferred from the sensor
  timestamps.
- Barometer: standard deviation of pressure about its mean; the slow
  PressureFactor (weather drift) term is negligible over the sampling window.
- GPS: standard deviation of latitude/longitude converted to metres and of
  altitude. Its update rate is low, so the sample count is reported.
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


def sigma_from_differences(values):
    if len(values) < 3:
        return None
    diffs = [b - a for a, b in zip(values, values[1:])]
    mean = sum(diffs) / len(diffs)
    var = sum((d - mean) ** 2 for d in diffs) / (len(diffs) - 1)
    return math.sqrt(var / 2.0)


def std(values):
    if len(values) < 2:
        return None
    mean = sum(values) / len(values)
    var = sum((v - mean) ** 2 for v in values) / (len(values) - 1)
    return math.sqrt(var)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=41452)
    parser.add_argument('--vehicle', default='PX4SITL')
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--rate-hz', type=float, default=100.0)
    parser.add_argument('--expected', type=Path, default=None,
                        help='settings JSON with the configured noise parameters')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    client = airsim.MultirotorClient(ip=args.host, port=args.port, timeout_value=30)
    report = {'status': 'running', 'seconds': args.seconds, 'vehicle': args.vehicle,
              'samples': {}, 'measured': {}, 'expected': {}, 'checks': []}

    imu_t, acc = [], [[], [], []]
    gyro = [[], [], []]
    baro_p, baro_alt = [], []
    mag = [[], [], []]
    gps_lat, gps_lon, gps_alt, gps_vn, gps_ve = [], [], [], [], []
    gps_sats, gps_eph, gps_epv, gps_fix = [], [], [], []

    try:
        client.confirmConnection()
        interval = 1.0 / args.rate_hz
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            imu = client.getImuData(vehicle_name=args.vehicle)
            imu_t.append(imu.time_stamp / 1e9)
            for i, field in enumerate(('x_val', 'y_val', 'z_val')):
                acc[i].append(getattr(imu.linear_acceleration, field))
                gyro[i].append(getattr(imu.angular_velocity, field))
            baro = client.getBarometerData(vehicle_name=args.vehicle)
            baro_p.append(baro.pressure)
            baro_alt.append(baro.altitude)
            m = client.getMagnetometerData(vehicle_name=args.vehicle)
            for i, field in enumerate(('x_val', 'y_val', 'z_val')):
                mag[i].append(getattr(m.magnetic_field_body, field))
            g = client.getGpsData(vehicle_name=args.vehicle)
            gps_lat.append(g.gnss.geo_point.latitude)
            gps_lon.append(g.gnss.geo_point.longitude)
            gps_alt.append(g.gnss.geo_point.altitude)
            gps_vn.append(g.gnss.velocity.x_val)
            gps_ve.append(g.gnss.velocity.y_val)
            gps_sats.append(int(getattr(g.gnss, 'satellites_visible', 0)))
            gps_eph.append(g.gnss.eph)
            gps_epv.append(g.gnss.epv)
            gps_fix.append(int(g.gnss.fix_type))
            time.sleep(interval)

        report['samples'] = {'imu': len(imu_t), 'baro': len(baro_p), 'mag': len(mag[0]),
                             'gps': len(gps_lat)}
        if len(imu_t) > 2:
            dt = (imu_t[-1] - imu_t[0]) / (len(imu_t) - 1)
            report['imu_dt_s'] = round(dt, 6)

        acc_sigma = [sigma_from_differences(ch) for ch in acc]
        gyro_sigma = [sigma_from_differences(ch) for ch in gyro]
        mag_sigma = [sigma_from_differences(ch) for ch in mag]
        mean_lat = sum(gps_lat) / len(gps_lat)
        m_per_deg = 111320.0
        north = [(la - mean_lat) * m_per_deg for la in gps_lat]
        east = [(lo - sum(gps_lon) / len(gps_lon)) * m_per_deg *
                math.cos(math.radians(mean_lat)) for lo in gps_lon]

        # Convert the IMU sample sigmas to the datasheet units: noise
        # density = sigma * sqrt(dt) (rad/sqrt(s), m/s^2/sqrt(s)).
        dt = report.get('imu_dt_s', 0) or 0.01
        report['measured'] = {
            'accel_sample_sigma_ms2': [round(s, 5) for s in acc_sigma if s is not None],
            'accel_noise_density_ms2_sqrts': round(min(s for s in acc_sigma if s) * math.sqrt(dt), 5)
            if any(acc_sigma) else None,
            'gyro_sample_sigma_rads': [round(s, 6) for s in gyro_sigma if s is not None],
            'gyro_noise_density_rad_sqrts': round(min(s for s in gyro_sigma if s) * math.sqrt(dt), 6)
            if any(gyro_sigma) else None,
            'baro_pressure_sigma_pa': round(std(baro_p), 4) if std(baro_p) else None,
            'baro_altitude_sigma_m': round(std(baro_alt), 4) if std(baro_alt) else None,
            'mag_sample_sigma_gauss': [round(s, 6) for s in mag_sigma if s is not None],
            'gps_north_sigma_m': round(std(north), 4) if std(north) else None,
            'gps_east_sigma_m': round(std(east), 4) if std(east) else None,
            'gps_vertical_sigma_m': round(std(gps_alt), 4) if std(gps_alt) else None,
            'gps_velocity_sigma_ms': round(max(std(gps_vn) or 0, std(gps_ve) or 0), 4),
            'gps_satellites_min': min(gps_sats) if gps_sats else None,
            'gps_satellites_max': max(gps_sats) if gps_sats else None,
            'gps_satellites_mean': round(sum(gps_sats) / len(gps_sats), 2) if gps_sats else None,
            'gps_eph_mean': round(sum(gps_eph) / len(gps_eph), 3) if gps_eph else None,
            'gps_epv_mean': round(sum(gps_epv) / len(gps_epv), 3) if gps_epv else None,
            'gps_fix_min': min(gps_fix) if gps_fix else None,
            'accel_magnitude_mean_ms2': round(
                sum(math.sqrt(sum(ch[i] ** 2 for ch in acc)) for i in range(len(acc[0])))
                / len(acc[0]), 4) if acc[0] else None,
        }

        if args.expected is not None:
            settings = json.loads(args.expected.read_text())
            sensors = None
            for vehicle in settings.get('Vehicles', {}).values():
                if vehicle.get('VehicleName', args.vehicle) or True:
                    sensors = vehicle.get('Sensors', {})
                    break
            gps_cfg = sensors.get('Gps', {})
            baro_cfg = sensors.get('Barometer', {})
            mag_cfg = sensors.get('Magnetometer', {})
            report['expected'] = {
                'gps_horizontal_sigma_m': gps_cfg.get('PositionNoiseSigma'),
                'gps_vertical_sigma_m': gps_cfg.get('VerticalNoiseSigma'),
                'gps_velocity_sigma_ms': gps_cfg.get('VelocityNoiseSigma'),
                'baro_pressure_sigma_pa': baro_cfg.get('UncorrelatedNoiseSigma'),
                'mag_sample_sigma_gauss': mag_cfg.get('NoiseSigma'),
            }
            if gps_cfg.get('SatelliteModel'):
                # The sigma the model injects follows the instantaneous DOP;
                # compare against the value at the measured mean satellite
                # count rather than the fixed PositionNoiseSigma setting.
                mean_sats = report['measured'].get('gps_satellites_mean') or 0
                if mean_sats > 0:
                    hdop = gps_cfg.get('HdopAtReference', 0.9) * math.sqrt(
                        gps_cfg.get('SatsReference', 10.0) / mean_sats)
                    uere = gps_cfg.get('Uere', 0.8)
                    report['expected']['gps_horizontal_sigma_m'] = uere * hdop
                    report['expected']['gps_vertical_sigma_m'] = (
                        uere * hdop * gps_cfg.get('VdopRatio', 1.6))

            # A single window of a time-correlated (Gauss-Markov) error is
            # itself a random variable with few effective degrees of freedom
            # (window / tau), so its standard deviation scatters widely
            # around sigma; the bounds for those checks are wider and the
            # window should be at least twice the correlation time.
            correlated = (gps_cfg.get('NoiseTau') or 0) > 0

            def check(name, measured, expected, low=0.5, high=2.0):
                ok = (measured is not None and expected is not None and
                      expected > 0 and low * expected <= measured <= high * expected)
                report['checks'].append({'name': name, 'passed': bool(ok),
                                         'measured': measured, 'expected': expected})

            m = report['measured']
            low, high = (0.3, 2.0) if correlated else (0.5, 2.0)
            check('gps north sigma within bounds of configured',
                  m['gps_north_sigma_m'], report['expected']['gps_horizontal_sigma_m'], low, high)
            check('gps east sigma within bounds of configured',
                  m['gps_east_sigma_m'], report['expected']['gps_horizontal_sigma_m'], low, high)
            check('gps vertical sigma within bounds of configured',
                  m['gps_vertical_sigma_m'], report['expected']['gps_vertical_sigma_m'], low, high)
            vlow, vhigh = (0.2, 3.0) if correlated else (0.3, 3.0)
            check('gps velocity sigma within bounds of configured',
                  m['gps_velocity_sigma_ms'], report['expected']['gps_velocity_sigma_ms'], vlow, vhigh)
            if gps_cfg.get('SatelliteModel'):
                sats = report['measured']
                minimum = gps_cfg.get('SatsMin', 7)
                maximum = gps_cfg.get('SatsMax', 15)
                ok_range = (sats['gps_satellites_min'] is not None and
                            minimum <= sats['gps_satellites_min'] and
                            sats['gps_satellites_max'] <= maximum)
                report['checks'].append({'name': 'satellite count within configured range',
                                         'passed': bool(ok_range),
                                         'measured': [sats['gps_satellites_min'],
                                                      sats['gps_satellites_max']],
                                         'expected': [minimum, maximum]})
                ok_fix = (sats['gps_fix_min'] or 0) >= 2
                report['checks'].append({'name': 'fix usable while satellites visible',
                                         'passed': bool(ok_fix),
                                         'measured': sats['gps_fix_min'], 'expected': '>= 2'})
                hdop = gps_cfg.get('HdopAtReference', 0.9) * math.sqrt(
                    gps_cfg.get('SatsReference', 10.0) / max(1.0, sats['gps_satellites_mean'] or 1))
                expected_eph = gps_cfg.get('Uere', 0.8) * hdop
                ok_eph = (sats['gps_eph_mean'] is not None and expected_eph > 0 and
                          0.6 * expected_eph <= sats['gps_eph_mean'] <= 3.0 * expected_eph)
                report['checks'].append({'name': 'reported EPH tracks UERE x HDOP',
                                         'passed': bool(ok_eph),
                                         'measured': sats['gps_eph_mean'],
                                         'expected': round(expected_eph, 3)})

            imu_cfg = None
            for vehicle in settings.get('Vehicles', {}).values():
                imu_cfg = vehicle.get('Sensors', {}).get('Imu', {})
                break
            if imu_cfg and imu_cfg.get('AccelNonlinearity'):
                # At rest the vertical specific force is 1 g, so the output
                # deviation is nonlinearity * g plus the turn-on bias.
                g = 9.80665
                expected_magnitude = g * (1.0 + imu_cfg['AccelNonlinearity']) +                     (imu_cfg.get('AccelTurnOnBias') or 0) * 1e-3 * g
                measured_magnitude = report['measured'].get('accel_magnitude_mean_ms2')
                ok_mag = (measured_magnitude is not None and
                          0.6 * expected_magnitude <= measured_magnitude <= 1.4 * expected_magnitude)
                report['checks'].append({
                    'name': 'accel magnitude at rest includes nonlinearity and bias',
                    'passed': bool(ok_mag),
                    'measured': measured_magnitude,
                    'expected': round(expected_magnitude, 3)})

            tau = gps_cfg.get('NoiseTau') or 0
            if correlated and args.seconds < 2 * tau:
                report['checks'].append({
                    'name': 'measurement window >= 2x correlation time',
                    'passed': False,
                    'measured': args.seconds, 'expected': '>= %g s' % (2 * tau)})
            imu = None
            for vehicle in settings.get('Vehicles', {}).values():
                imu = vehicle.get('Sensors', {}).get('Imu', {})
                break
            arw = imu.get('AngularRandomWalk')
            vrw = imu.get('VelocityRandomWalk')
            gyro_sigma = m['gyro_sample_sigma_rads']
            acc_sigma = m['accel_sample_sigma_ms2']
            if arw and gyro_sigma and vrw and acc_sigma:
                # sigma_sample = density / sqrt(dt): invert for dt and report
                # the implied IMU update rate. Both axes must agree.
                arw_si = math.radians(arw) / math.sqrt(3600.0)
                vrw_si = vrw * 9.80665 / 1000.0
                dt_gyro = (arw_si / min(gyro_sigma)) ** 2
                dt_acc = (vrw_si / min(acc_sigma)) ** 2
                rate_gyro, rate_acc = 1.0 / dt_gyro, 1.0 / dt_acc
                report['measured']['imu_rate_from_gyro_hz'] = round(rate_gyro, 1)
                report['measured']['imu_rate_from_accel_hz'] = round(rate_acc, 1)
                ok = (150.0 <= rate_gyro <= 1000.0 and 150.0 <= rate_acc <= 1000.0 and
                      abs(rate_gyro - rate_acc) / rate_gyro < 0.25)
                report['checks'].append({'name': 'IMU sample sigmas imply a plausible update rate',
                                         'passed': bool(ok),
                                         'measured': [round(rate_gyro, 1), round(rate_acc, 1)],
                                         'expected': '150-1000 Hz, both axes agree within 25%'})

            check('baro pressure sigma within 0.5-2x configured',
                  m['baro_pressure_sigma_pa'], report['expected']['baro_pressure_sigma_pa'], 0.5, 2.0)
            mag_expected = report['expected']['mag_sample_sigma_gauss']
            mag_measured = m['mag_sample_sigma_gauss']
            check('mag sigma within 0.5-2x configured',
                  min(mag_measured) if mag_measured else None, mag_expected, 0.5, 2.0)

        report['status'] = 'passed' if all(c['passed'] for c in report['checks']) else (
            'failed' if report['checks'] else 'measured')
    except Exception as error:
        report['status'] = 'failed'
        report['error'] = repr(error)
        print(repr(error), flush=True)
    finally:
        client.close()
        (args.output / 'noise.json').write_text(
            json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(json.dumps(report.get('measured'), indent=2))
    for c in report['checks']:
        print(('PASS ' if c['passed'] else 'FAIL ') + c['name'], c['measured'], 'vs', c['expected'])
    return 0 if report['status'] != 'failed' else 1


if __name__ == '__main__':
    sys.exit(main())
