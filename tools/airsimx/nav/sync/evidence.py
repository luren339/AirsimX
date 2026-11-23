"""Shared recording contract and ROS-independent evidence checks for Blocks SITL."""
from collections import Counter, defaultdict
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import struct

NAV = '/drone_city_nav/'
CLOCK = '/clock'
BASE = NAV + 'raw_obstacle_snapshot_3d'
DELTA = NAV + 'raw_obstacle_delta_3d'
MEMORY = NAV + 'obstacle_memory_status'
DIAG = NAV + 'mppi/status'
SCAN = NAV + 'latest_sensor_obstacle_scan'
VEHICLE = NAV + 'vehicle_state'
CONTROL = NAV + 'mppi/applied_control'
OBJECTIVE = NAV + 'navigation_objective'
LIDAR = '/lidar_3d/points'
NODE = '/production_mppi_node'
PX4 = tuple('/fmu/out/' + n for n in ('vehicle_local_position_v1', 'vehicle_status_v1', 'vehicle_attitude'))
REQUIRED = (CLOCK, *PX4, VEHICLE, BASE, DELTA, MEMORY, SCAN, DIAG, CONTROL)
OPTIONAL = (LIDAR, OBJECTIVE, NAV + 'mission_waypoint_acknowledgement', NAV + 'mission_events',
            NAV + 'truth/odom', '/barometer/air_pressure', '/magnetometer/magnetic_field')
TOPICS = REQUIRED + OPTIONAL
TYPES = dict(zip(REQUIRED, ('rosgraph_msgs/msg/Clock', 'px4_msgs/msg/VehicleLocalPosition',
    'px4_msgs/msg/VehicleStatus', 'px4_msgs/msg/VehicleAttitude', 'drone_city_nav/msg/VehicleNavigationState',
    'drone_city_nav/msg/RawObstacleSnapshot3D', 'drone_city_nav/msg/RawObstacleDelta3D',
    'drone_city_nav/msg/ObstacleMemoryStatus', 'drone_city_nav/msg/LatestSensorObstacleScan',
    'std_msgs/msg/String', 'drone_city_nav/msg/MppiControlFeedback')))
TYPES.update({LIDAR: 'sensor_msgs/msg/PointCloud2', OBJECTIVE: 'drone_city_nav/msg/NavigationObjective'})
NS = 1_000_000_000
MASK = (1 << 64) - 1


def stamp(t):
    return int(t.sec) * NS + int(t.nanosec)


def sha(path):
    with Path(path).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def dump(path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    tmp.replace(path)


def fingerprint(geometry, occupied_chunks):
    # The planner hashes little-endian geometry and sorted nonempty occupied
    # chunks, not observed-only cells or the CDR serialization of the message.
    data = bytearray(struct.pack('<4d4Q', *(0.0 if x == 0 else x for x in geometry[:4]),
                                 *geometry[4:7], len(occupied_chunks)))
    for xyz, words in sorted(occupied_chunks.items()):
        data.extend(struct.pack('<3q64Q', *xyz, *words))
    h = 1469598103934665603
    for b in data:
        h = ((h ^ b) * 1099511628211) & MASK
    return str(h or 1)


@dataclass(frozen=True)
class World:
    producer: int
    revision: int
    base: int
    snapshot: bool
    source_ns: int
    receipt_ns: int
    occupied: int
    fingerprint: str
    cdr_sha256: str


class Worlds:
    def __init__(self):
        self.bases = {}
        self.rows = defaultdict(list)
        self.statuses = defaultdict(set)
        self.producers = set()
        self.skipped = Counter()

    def add_status(self, m, received):
        p = int(m.producer_instance_id)
        self.producers.add(p)
        self.statuses[p, stamp(m.header.stamp), m.header.frame_id].add((
            bool(m.raw_snapshot_published), bool(m.raw_delta_published), bool(m.full_snapshot_published),
            int(m.occupied_cell_count), int(m.sequence), received))

    def add_map(self, topic, m, received, raw):
        p, rev = int(m.producer_instance_id), int(m.obstacle_snapshot_revision)
        self.producers.add(p)
        base = rev if topic == BASE else int(m.base_snapshot_revision)
        geo = (m.origin_x_m, m.origin_y_m, m.origin_z_m, m.resolution_m, m.width_cells,
               m.height_cells, m.depth_cells, m.chunk_size_cells)
        if (not all(map(math.isfinite, geo[:4])) or geo[3] <= 0 or min(geo[4:7]) <= 0 or
                geo[-1] != 16 or m.header.frame_id != 'map' or not p or not rev):
            raise ValueError('unsupported map identity, frame or geometry')
        chunks = {}
        if topic == DELTA:
            ref = self.bases.get((p, base))
            if ref is None:
                self.skipped['missing_base'] += 1
                return
            if geo != ref[0] or rev < base:
                raise ValueError('delta geometry/revision differs from named base')
            # Each delta replaces whole dirty chunks relative to its named BASE,
            # not relative to the preceding delta.
            chunks = ref[1].copy()
        seen = set()
        for c in m.chunks:
            xyz = (c.x, c.y, c.z)
            words = tuple(c.occupied_words)
            if xyz in seen or len(words) != 64 or len(c.observed_words) != 64:
                raise ValueError('duplicate or malformed map chunk')
            seen.add(xyz)
            if any(words):
                chunks[xyz] = words
            else:
                chunks.pop(xyz, None)
        digest = hashlib.sha256(raw).hexdigest()
        if topic == BASE:
            self.bases[p, base] = (geo, chunks)
        row = World(p, rev, base, topic == BASE, stamp(m.header.stamp), received,
                    sum(w.bit_count() for words in chunks.values() for w in words),
                    fingerprint(geo, chunks), digest)
        self.rows[p, rev].append(row)

    def _attested(self, row, when):
        entries = self.statuses[row.producer, row.source_ns, 'map']
        flags = (row.snapshot, not row.snapshot, row.snapshot, row.occupied)
        claims = {(s[:4], s[4]) for s in entries if s[-1] <= when and any(s[:2])}
        if len(claims) == 1:
            attestation, seq = next(iter(claims))
            if attestation == flags:
                return seq
        return None

    def match(self, rev, fp, when):
        # The public planner line has no raw-map producer ID. Do not guess across
        # epochs, even when two producers happen to publish identical content.
        if len(self.producers) != 1:
            return None, 'producer_missing_or_ambiguous'
        p, = self.producers
        rows = self.rows.get((p, rev), [])
        if len({r.cdr_sha256 for r in rows}) != 1:
            return None, 'resident_revision_absent_or_conflicting'
        r = min(rows, key=lambda x: x.receipt_ns)
        if r.fingerprint != fp or not r.occupied:
            return None, 'fingerprint_mismatch_or_empty'
        bases = [b for b in self.rows[p, r.base] if b.snapshot]
        if len({b.cdr_sha256 for b in bases}) != 1:
            return None, 'base_absent_or_conflicting'
        b = min(bases, key=lambda x: x.receipt_ns)
        if max(r.receipt_ns, r.source_ns, b.receipt_ns, b.source_ns) > when:
            return None, 'map_not_yet_recorded'
        seq = self._attested(r, when)
        base_seq = self._attested(b, when)
        if seq is None or base_seq is None:
            return None, 'missing_matching_producer_status'
        return {'producer_instance_id': str(p), 'status_sequence': seq, 'resident_revision': rev,
                'base_revision': r.base, 'base_status_sequence': base_seq, 'occupied_fingerprint': fp,
                'occupied_voxels': r.occupied, 'map_stamp_ns': r.source_ns,
                'map_bag_ns': r.receipt_ns, 'base_cdr_sha256': b.cdr_sha256, 'map_cdr_sha256': r.cdr_sha256}, None


def search_sample(line):
    """Read one complete published diagnostics line; never carry fields across ticks."""
    tokens = line.split()
    if not tokens or tokens[0] != 'PRODUCTION_MPPI_TICK':
        return None
    pairs = [t.split('=', 1) for t in tokens[1:] if '=' in t]
    d = dict(pairs)
    if len(d) != len(pairs):
        return None
    try:
        epoch, sample, rev, expansions = map(int, (d['route_objective_epoch'], d['route_objective_sample'],
                                                   d['planner_planned_on_revision'], d['planner_expansions']))
        if (d['planning_search_kind'] != 'persistent_dstar_lite_3d' or d['planner_invoked'] != 'true' or
                d['planner_input'] != 'accepted' or d['planner_candidate_published'] != 'true' or
                min(epoch, rev, expansions) < 1 or sample < 0 or int(d['planner_mission_epoch']) != epoch or
                int(d['route_intent_planned_on']) != rev or not 0 < int(d['planner_occupied_fingerprint']) <= MASK):
            return None
        g = d['planning_search_goal']
        if not (g.startswith('(') and g.endswith(')')):
            return None
        goal = [float(x) for x in g[1:-1].split(',')]
        if len(goal) != 3 or not all(map(math.isfinite, goal)):
            return None
        return {'tick': int(d['tick']), 'mission_epoch': epoch, 'objective_sample': sample,
                'published_search_mission_goal_map_m': goal, 'resident_revision': rev,
                'occupied_fingerprint': str(int(d['planner_occupied_fingerprint'])),
                'dstar_expansions': expansions, 'raw_revision_at_tick': int(d['raw_revision'])}
    except (KeyError, ValueError):
        return None
