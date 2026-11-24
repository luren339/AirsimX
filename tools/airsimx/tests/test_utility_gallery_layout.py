"""Topology/clearance regressions migrated from the source project."""

import json
from pathlib import Path
import sys
import tempfile
import unittest

from _generated_geometry_test_utils import TOOLS

sys.path.insert(0, str(TOOLS / 'scenes'))
from utility_gallery_layout import DEFAULT_CONFIG, build_layout


class GalleryLayoutTests(unittest.TestCase):
    def config(self):
        return json.loads(DEFAULT_CONFIG.read_text(encoding='utf-8'))

    def test_four_connected_loops_and_two_dead_end_service_spurs(self):
        layout = build_layout()
        adjacency = {node['id']: set() for node in layout['nodes']}
        for edge in layout['edges']:
            adjacency[edge['from']].add(edge['to'])
            adjacency[edge['to']].add(edge['from'])
        visited, stack = set(), ['J00']
        while stack:
            node = stack.pop()
            if node not in visited:
                visited.add(node)
                stack.extend(adjacency[node] - visited)
        self.assertEqual(len(visited), 9)
        self.assertEqual(len(layout['edges']) - len(visited) + 1, 4)
        self.assertEqual(len(layout['branches']), 2)
        self.assertEqual([b['length_m'] for b in layout['branches']], [10, 10])

    def test_current_dimensions_and_clear_flight_envelope(self):
        layout = build_layout()
        self.assertEqual(layout['dimensions_m']['corridor_width'], 4.0)
        self.assertEqual(layout['dimensions_m']['interior_height'], 3.6)
        self.assertEqual(layout['dimensions_m']['junction_width'], 5.0)
        self.assertEqual(layout['dimensions_m']['junction_height'], 3.6)
        self.assertEqual(layout['flight_envelope_m'], {'width': 2, 'height': 2.2, 'center_height': 1.8})

    def test_deterministic_uneven_cells_and_explicit_config_path(self):
        layout = build_layout()
        self.assertEqual(layout, build_layout())
        xs = sorted({n['position_m'][0] for n in layout['nodes']})
        ys = sorted({n['position_m'][1] for n in layout['nodes']})
        self.assertEqual([xs[1] - xs[0], xs[2] - xs[1]], [28, 33])
        self.assertEqual([ys[1] - ys[0], ys[2] - ys[1]], [22, 25])
        with tempfile.TemporaryDirectory() as directory:
            cfg = self.config()
            cfg['columns_x_m'] = [-30, 0, 35]
            path = Path(directory) / 'layout.json'
            path.write_text(json.dumps(cfg))
            self.assertEqual(build_layout(path)['nodes'][0]['position_m'], [-30, -22])

    def test_corridors_meet_room_faces_without_gaps(self):
        layout = build_layout()
        nodes = {n['id']: n['position_m'] for n in layout['nodes']}
        for edge in layout['edges']:
            a, b = nodes[edge['from']], nodes[edge['to']]
            start, end = edge['start_m'], edge['end_m']
            self.assertAlmostEqual(sum((start[i] - a[i]) ** 2 for i in (0, 1)) ** 0.5, 2.5)
            self.assertAlmostEqual(sum((end[i] - b[i]) ** 2 for i in (0, 1)) ** 0.5, 2.5)
            self.assertAlmostEqual(sum((start[i] - end[i]) ** 2 for i in (0, 1)) ** 0.5, edge['length_m'])

    def test_invalid_layout_values_fail_before_geometry_generation(self):
        changes = [
            ('columns_x_m', [0, 0, 20]),
            ('rows_y_m', [0, float('nan'), 30]),
            ('dimensions_m', {'corridor_width': 1.5}),
            ('dimensions_m', {'shell_thickness': -0.1}),
            ('flight_envelope_m', {'center_height': 0.1}),
            ('fixture_spacing_m', {'light_min': 0}),
            ('fixture_spacing_m', {'light_max': 4.6}),
            ('branches', [{'id': 'bad', 'from_node': 'J00', 'direction': 'west', 'length_m': 0}]),
        ]
        for key, values in changes:
            with self.subTest(key=key, values=values):
                cfg = self.config()
                if isinstance(values, dict):
                    cfg[key].update(values)
                else:
                    cfg[key] = values
                with self.assertRaises(ValueError):
                    build_layout(cfg)

    def test_fixture_spacing_that_quantizes_to_zero_is_rejected_without_generating(self):
        cfg = self.config()
        cfg['fixture_spacing_m'].update(cable_support_min=0.0001, cable_support_max=0.0002)
        with self.assertRaises(ValueError):
            build_layout(cfg)

    def test_branches_cannot_reuse_corridor_exits_branch_exits_or_ids(self):
        cases = (
            [{'id': 'BlockedExit', 'from_node': 'J11', 'direction': 'east', 'length_m': 10}],
            [{'id': 'First', 'from_node': 'J00', 'direction': 'west', 'length_m': 10},
             {'id': 'Second', 'from_node': 'J00', 'direction': 'west', 'length_m': 10}],
            [{'id': 'Same', 'from_node': 'J00', 'direction': 'west', 'length_m': 10},
             {'id': 'Same', 'from_node': 'J00', 'direction': 'south', 'length_m': 10}],
            [{'id': 'H00', 'from_node': 'J00', 'direction': 'west', 'length_m': 10}],
        )
        for branches in cases:
            with self.subTest(branches=branches):
                cfg = self.config()
                cfg['branches'] = branches
                with self.assertRaises(ValueError):
                    build_layout(cfg)

    def test_junction_must_be_strictly_wider_than_corridors_and_branches(self):
        for changes in ({'junction_width': 4}, {'branch_width': 5}, {'branch_width': 6}):
            with self.subTest(changes=changes):
                cfg = self.config()
                cfg['dimensions_m'].update(changes)
                with self.assertRaises(ValueError):
                    build_layout(cfg)


if __name__ == '__main__':
    unittest.main()
