"""Offline tests for the passive ROS recording evidence contract, without ROS or a flight."""
from pathlib import Path
import sys
from types import SimpleNamespace as Obj
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'nav' / 'sync'))
from evidence import BASE, CLOCK, DELTA, NS, Worlds, search_sample
from verify import Timeline


def grid(rev, chunks, *, producer=7, base=1, at=None):
    header = Obj(frame_id='map', stamp=Obj(sec=0, nanosec=at or rev * 10))
    return Obj(header=header, producer_instance_id=producer, obstacle_snapshot_revision=rev,
               base_snapshot_revision=base, origin_x_m=0., origin_y_m=0., origin_z_m=0., resolution_m=.25,
               width_cells=64, height_cells=64, depth_cells=64, chunk_size_cells=16,
               chunks=[Obj(x=k, y=0, z=0, occupied_words=[v] + [0] * 63,
                           observed_words=[7] + [0] * 63) for k, v in chunks])


def status(m, count, full=True):
    return Obj(header=m.header, producer_instance_id=m.producer_instance_id, raw_snapshot_published=full,
               raw_delta_published=not full, full_snapshot_published=full, occupied_cell_count=count,
               sequence=m.obstacle_snapshot_revision + 90)  # status sequence need not equal revision


LINE = ('PRODUCTION_MPPI_TICK tick=9 planner_invoked=true planner_candidate_published=true '
        'planner_input=accepted planning_search_kind=persistent_dstar_lite_3d route_intent_planned_on=3 '
        'route_objective_epoch=1 route_objective_sample=0 planner_mission_epoch=1 planner_planned_on_revision=3 '
        'raw_revision=4 planner_occupied_fingerprint=123 planner_expansions=5 planner_feasibility_expansions=90 '
        'planning_candidate_endpoint=(8,1,2) target=(5,5,5) planning_search_goal=(10,0,3)')


class SearchDiagnosticsTests(unittest.TestCase):
    def test_mission_goal_and_resident_revision_not_other_fields(self):
        s = search_sample(LINE)
        self.assertEqual(s['published_search_mission_goal_map_m'], [10, 0, 3])
        self.assertEqual((s['resident_revision'], s['raw_revision_at_tick']), (3, 4))

    def test_refused_or_incoherent_samples(self):
        for old, new in [('expansions=5 ', 'expansions=0 '), ('input=accepted', 'input=pending'),
                         ('planner_mission_epoch=1', 'planner_mission_epoch=2'),
                         ('planned_on=3 ', 'planned_on=2 '), ('candidate_published=true', 'candidate_published=false'),
                         ('(10,0,3)', '(nan,0,3)'), ('planning_search_goal', 'unused_goal'),
                         ('planner_expansions=5', 'planner_expansions=5 planner_expansions=0')]:
            with self.subTest(old=old):
                self.assertIsNone(search_sample(LINE.replace(old, new)))

    def test_do_not_stitch_separate_diagnostics(self):
        before, after = LINE.split('planning_search_goal')
        self.assertIsNone(search_sample(before))
        self.assertIsNone(search_sample('PRODUCTION_MPPI_TICK planning_search_goal' + after))


class TimelineTests(unittest.TestCase):
    def test_clock_reset_not_hidden_by_timestamp_sort(self):
        t = Timeline()
        for n in (3, 4, 1, 2):
            t.add(CLOCK, n * NS, n * NS)
        t.prepare()
        self.assertEqual(t.clock_backsteps, 1)

    def test_recent_delivery_is_not_recent_acquisition(self):
        t = Timeline()
        t.add('state', 3 * NS, NS)
        t.add('state', 3 * NS + 1, 3 * NS + 1)
        t.prepare()
        self.assertIsNone(t.near('state', 3 * NS))
        self.assertIsNotNone(t.near('state', 3 * NS + 1))


class WorldEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.w = Worlds()
        self.b = grid(1, [(0, 1), (1, 2)])
        self.w.add_map(BASE, self.b, 10, b'base')
        self.w.add_status(status(self.b, 2), 11)

    def test_cumulative_delta_reverts_to_base_and_clears_chunks(self):
        self.w.add_map(DELTA, grid(2, [(0, 4)]), 20, b'first')
        d = grid(3, [(1, 0)])
        self.w.add_map(DELTA, d, 30, b'second')
        self.w.add_status(status(d, 1, False), 31)
        ref = Worlds()
        ref.add_map(BASE, grid(3, [(0, 1)]), 30, b'full')
        fp = ref.rows[7, 3][0].fingerprint
        witness, reason = self.w.match(3, fp, 40)
        self.assertIsNone(reason)
        self.assertEqual((witness['base_revision'], witness['status_sequence']), (1, 93))
        self.assertNotEqual(witness['map_cdr_sha256'], fp)

    def test_one_moved_voxel_invalidates_equal_count(self):
        other = Worlds()
        other.add_map(BASE, grid(1, [(0, 4), (1, 2)]), 10, b'different')
        self.assertIsNone(self.w.match(1, other.rows[7, 1][0].fingerprint, 15)[0])

    def test_missing_base_is_not_an_empty_base(self):
        self.w.add_map(DELTA, grid(3, [(0, 1)], base=2), 33, b'orphan')
        self.assertEqual(self.w.skipped['missing_base'], 1)
        self.assertIsNone(self.w.match(3, '10', 40)[0])

    def test_wrong_or_missing_producer_attestation(self):
        d = grid(3, [(0, 4)])
        self.w.add_map(DELTA, d, 30, b'delta')
        fp = self.w.rows[7, 3][0].fingerprint
        for s in (status(grid(3, [], at=29), 2, False), status(d, 0, False), status(d, 2, True)):
            self.w.add_status(s, 31)
            self.assertIsNone(self.w.match(3, fp, 40)[0])

    def test_future_status_not_available_at_tick(self):
        fp = self.w.rows[7, 1][0].fingerprint
        self.assertIsNone(self.w.match(1, fp, 10)[0])
        self.assertIsNotNone(self.w.match(1, fp, 11)[0])

    def test_second_epoch_makes_implicit_producer_ambiguous(self):
        self.w.add_map(BASE, grid(1, [(0, 1), (1, 2)], producer=8), 12, b'other')
        self.assertIsNone(self.w.match(1, self.w.rows[7, 1][0].fingerprint, 15)[0])

    def test_conflicting_bases_are_not_interchangeable(self):
        d = grid(3, [(1, 0)])
        self.w.add_map(DELTA, d, 30, b'delta')
        self.w.add_status(status(d, 1, False), 30)
        self.w.add_map(BASE, self.b, 10, b'conflicting base CDR')
        self.assertIsNone(self.w.match(3, self.w.rows[7, 3][0].fingerprint, 40)[0])


if __name__ == '__main__':
    unittest.main()
