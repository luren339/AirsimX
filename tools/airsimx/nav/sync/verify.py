"""Read a finalized capture without opening a ROS graph. Exit 0 = at least one witnessed search, 2 = incomplete."""
import argparse
from bisect import bisect_right
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys

from evidence import (BASE, CLOCK, CONTROL, DELTA, DIAG, LIDAR, MEMORY, NODE, NS, OBJECTIVE, OPTIONAL,
                      PX4, REQUIRED, SCAN, TOPICS, TYPES, VEHICLE, Worlds, search_sample, sha, stamp)


class Timeline:
    def __init__(self):
        self.rows = defaultdict(list)
        self.clock_backsteps = 0

    def add(self, topic, received, source):
        rows = self.rows[topic]
        if topic == CLOCK and rows and source < rows[-1][1]:
            self.clock_backsteps += 1
        rows.append((received, source))

    def prepare(self):
        for rows in self.rows.values():
            rows.sort()

    def near(self, topic, when):
        # An evidence window, not a change to any navigation freshness gate.
        rows = self.rows[topic]
        i = bisect_right(rows, (when, math.inf))
        for j in range(i - 1, -1, -1):
            recv, src = rows[j]
            if when - recv > NS:
                break
            if 0 < src <= when and when - src <= NS:
                return {'bag_ns': recv, 'source_ns': src}
        return None


def load_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def check_provenance(run, report):
    import yaml

    m = load_json(run / 'manifest.json')
    errors = report['incomplete_reasons']
    report['scene'] = m['scene']
    if m.get('format') != 1 or m.get('state') != 'closed' or m.get('recorder_exit') != 0:
        errors.append('recorder did not finalize cleanly')
    required_files = {'qos.yaml', 'source-params.yaml'}
    if m['scene'] == 'blocks':
        required_files.add('airsim-settings.json')
    for name in required_files:
        if sha(run / name) != m['inputs'][name]['sha256']:
            raise ValueError('input hash mismatch: ' + name)
    bag = run / 'bag'
    metadata = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    segments = set(metadata['relative_file_paths'])
    if (not segments or any(Path(s).name != s or not s.endswith('.mcap') for s in segments) or
            segments != {p.name for p in bag.glob('*.mcap')} or
            set(m['bag_sha256']) != segments | {'metadata.yaml'} or metadata['storage_identifier'] != 'mcap'):
        raise ValueError('MCAP inventory differs from manifest or metadata')
    for name, digest in m['bag_sha256'].items():
        if sha(bag / name) != digest:
            raise ValueError('bag hash mismatch: ' + name)
    report['bag_sha256'] = m['bag_sha256']
    report['navigation_commit'] = m['navigation_commit']
    report['installed_planner_sha256'] = m['installed_planner']['sha256']
    report['image_id'] = m['image_id']
    if sha(run / 'publishers.json') != m.get('publishers_sha256'):
        errors.append('unverified publisher inventory')
    graph = load_json(run / 'publishers.json')
    if [e['node'] for e in graph.get(DIAG, [])] != [NODE]:
        errors.append('missing or ambiguous production diagnostics publisher')
    for t in REQUIRED:
        if len(graph.get(t, [])) != 1:
            errors.append('expected a single observed publisher: ' + t)
    owners = {e['node'] for t in (BASE, DELTA, MEMORY) for e in graph.get(t, [])}
    if len(owners) != 1:
        errors.append('raw map and producer-status publishers differ')
    if m['scene'] == 'blocks':
        if owners != {'/obstacle_memory_3d_node'}:
            errors.append('Blocks profile requires the native obstacle-memory publisher, not a player')
        for t in (CLOCK, LIDAR):
            if [e['node'] for e in graph.get(t, [])] != ['/airsim_source']:
                errors.append('missing unique AirSim source publisher: ' + t)
    par = run / 'effective-planner.yaml'
    if not par.is_file() or sha(par) != m.get('effective_planner', {}).get('sha256'):
        errors.append('missing or unverified effective planner parameters')
    else:
        v = yaml.safe_load(par.read_text())[NODE]['ros__parameters']
        keys = ('mission_goal_sequence_xyz_m', 'use_sim_time',
                'px4_local_origin_x_m', 'px4_local_origin_y_m', 'px4_local_origin_z_m',
                'px4_to_map_m00', 'px4_to_map_m01', 'px4_to_map_m10', 'px4_to_map_m11')
        report['effective_parameters'] = {k: v[k] for k in keys}
        if v['use_sim_time'] is not True:
            errors.append('planner is not using simulation time')
        # Clock mapping is declared by the obstacle producer, NOT the planner.
        # Its input config is recorded separately; witness-time alignment below
        # uses measured (PX4 microseconds, ROS /clock, bag nanoseconds) tuples.
        source = yaml.safe_load((run / 'source-params.yaml').read_text())
        report['configured_px4_clock_mapping_by_node'] = {
            n: cfg['ros__parameters']['px4_clock_is_ros_clock'] for n, cfg in source.items()
            if isinstance(cfg, dict) and 'px4_clock_is_ros_clock' in cfg.get('ros__parameters', {})}
    return {x['topic_metadata']['name']: x['message_count'] for x in metadata['topics_with_message_count']}


def verify(run, report, expected):
    import rosbag2_py as rb
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    meta_counts = check_provenance(run, report)
    errors = report['incomplete_reasons']
    reader = rb.SequentialReader()
    reader.open(rb.StorageOptions(uri=str(run / 'bag'), storage_id='mcap'), rb.ConverterOptions('', ''))
    if not reader.set_read_order(rb.ReadOrder(rb.ReadOrderSortBy.File, False)):
        raise ValueError('need physical record order to detect /clock resets')
    types = {e.name: e.type for e in reader.get_all_topics_and_types()}
    for topic, name in types.items():
        if topic in TYPES and TYPES[topic] != name:
            raise ValueError(f'{topic}: unexpected type {name}')
    cls = {t: get_message(n) for t, n in types.items() if t in TYPES}
    worlds, times, samples, counts = Worlds(), Timeline(), [], Counter()
    objectives = []
    while reader.has_next():
        topic, raw, recv = reader.read_next()
        counts[topic] += 1
        if topic not in cls:
            continue
        msg = deserialize_message(raw, cls[topic])
        if topic in (BASE, DELTA):
            worlds.add_map(topic, msg, recv, raw)
        elif topic == MEMORY:
            worlds.add_status(msg, recv)
        elif topic == DIAG:
            s = search_sample(msg.data)
            if s:
                samples.append((recv, s, hashlib.sha256(raw).hexdigest()))
        elif topic == OBJECTIVE:
            objectives.append({'mission_epoch': msg.mission_epoch, 'sample_sequence': msg.sample_sequence,
                               'goal_map_m': [msg.position.x, msg.position.y, msg.position.z],
                               'source_ns': stamp(msg.stamp), 'bag_ns': recv})
        elif topic == CLOCK:
            times.add(topic, recv, stamp(msg.clock))
        elif topic in PX4:
            times.add(topic, recv, int(msg.timestamp) * 1000)  # PX4 microseconds, not nanoseconds.
        elif topic == VEHICLE:
            times.add(topic, recv, stamp(msg.stamp))
        elif topic in (SCAN, CONTROL, LIDAR):
            if topic == SCAN and not len(msg.hit_points_body_frd):
                continue
            if topic == LIDAR and not (msg.width * msg.height and len(msg.data)):
                continue
            times.add(topic, recv, stamp(msg.header.stamp))
    required = REQUIRED + ((LIDAR,) if report['scene'] == 'blocks' else ())
    report['topic_counts'] = {t: counts[t] for t in TOPICS}
    report['optional_absent'] = [t for t in OPTIONAL if not counts[t]]
    report['observed_dynamic_objectives'] = objectives
    for t in required:
        if not counts[t]:
            errors.append('no recorded messages: ' + t)
    if counts != Counter(meta_counts):
        errors.append('decoded message counts disagree with metadata')
    if times.clock_backsteps:
        errors.append(f'/clock moved backwards {times.clock_backsteps} times; separate runs required')
    times.prepare()
    timed = [CLOCK, *PX4, VEHICLE, SCAN, CONTROL] + ([LIDAR] if report['scene'] == 'blocks' else [])
    report['clock_and_state_ranges'] = {}
    for t in timed:
        rows = times.rows[t]
        stamps = {src for _, src in rows if src > 0}
        if len(stamps) < 2:
            errors.append('source timestamp did not advance: ' + t)
        if rows and stamps:
            report['clock_and_state_ranges'][t] = {'first_bag_ns': rows[0][0], 'last_bag_ns': rows[-1][0],
                'min_source_ns': min(stamps), 'max_source_ns': max(stamps), 'distinct_source_stamps': len(stamps)}
    report['map_producer_ids'] = sorted(str(x) for x in worlds.producers)
    report['map_decode_skips'] = dict(worlds.skipped)
    report['candidate_search_samples'] = len(samples)
    rejected = Counter()
    for recv, s, cdr in samples:
        world, reason = worlds.match(s['resident_revision'], s['occupied_fingerprint'], recv)
        if reason:
            rejected[reason] += 1
            continue
        coverage = {t: times.near(t, recv) for t in timed}
        if not all(coverage.values()):
            rejected['no_contemporaneous_state:' + ','.join(t for t, v in coverage.items() if v is None)] += 1
            continue
        report['witnesses'].append(dict(s, **{'world': world, 'diagnostic_bag_ns': recv,
                                               'diagnostic_cdr_sha256': cdr, 'nearby_samples': coverage}))
    report['rejected_samples'] = dict(rejected)
    if not report['witnesses']:
        errors.append('no single published D* search sample joined to an attested nonempty resident map and state')
    if expected:
        # The status formatter prints only three decimal places. Other goals are kept in the report.
        report['expected_goal_map_m'] = expected
        n = sum(all(abs(a - b) <= 0.000500001 for a, b in zip(w['published_search_mission_goal_map_m'], expected))
                for w in report['witnesses'])
        report['witnesses_with_expected_goal'] = n
        if not n:
            errors.append('no witnessed sample with the requested mission goal')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('run', type=Path)
    p.add_argument('--expect-goal', nargs=3, type=float, metavar=('X', 'Y', 'Z'))
    p.add_argument('--report', type=Path, help='new JSON file; default: print full report')
    args = p.parse_args()
    if args.expect_goal and not all(map(math.isfinite, args.expect_goal)):
        p.error('expected goal must be finite')
    r = {'status': 'incomplete', 'incomplete_reasons': [], 'witnesses': [],
         'scope': 'Sampled public candidate mission goal and D* work on a uniquely matching recorded world; '
                  'not the internal refined endpoint, every search, a certified route, flight, or arrival.'}
    try:
        verify(args.run, r, args.expect_goal)
    except Exception as err:
        r['incomplete_reasons'].append(f'{type(err).__name__}: {err}')
    if not r['incomplete_reasons']:
        r['status'] = 'search_witnessed'
    text = json.dumps(r, indent=2, allow_nan=False)
    if args.report:
        with args.report.open('x', encoding='utf-8') as f:
            f.write(text + '\n')
        print(r['status'], 'witnesses:', len(r['witnesses']), 'reasons:', r['incomplete_reasons'])
    else:
        print(text)
    return 0 if r['status'] == 'search_witnessed' else 2


if __name__ == '__main__':
    sys.exit(main())
