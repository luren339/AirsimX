"""Generate the real scene in a clean directory; inspect its binary geometry."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import cv2
import numpy as np

from _generated_geometry_test_utils import TOOLS, accessor_values, read_glb


class GalleryCLITests(unittest.TestCase):
    def test_scene_cli_generates_without_reference_assets_from_another_cwd(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'generated'
            result = subprocess.run(
                [sys.executable, str(TOOLS / 'scenes/generate_utility_gallery_glb.py'),
                 '--output-dir', str(output)], cwd=directory, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            metadata = json.loads((output / 'utility_gallery_geometry.json').read_text())
            self.assertEqual(metadata['source_assets_used'], [])
            self.assertEqual(metadata['textures']['mode'], 'procedural')
            document, binary = read_glb(output / 'utility_gallery_preview.glb')
            self.assertGreater(len(binary), 1000)
            self.assertEqual(document['asset']['version'], '2.0')


class GalleryGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.output = Path(cls.temporary.name)
        result = subprocess.run(
            [sys.executable, str(TOOLS / 'scenes/generate_utility_gallery_glb.py'),
             '--output-dir', str(cls.output)], capture_output=True, text=True)
        if result.returncode:
            raise AssertionError(result.stdout + result.stderr)
        cls.metadata = json.loads((cls.output / 'utility_gallery_geometry.json').read_text())
        cls.document, cls.binary = read_glb(cls.output / 'utility_gallery_preview.glb')

    def test_all_primitives_have_finite_geometry_valid_indices_and_matching_winding(self):
        triangle_count = 0
        for primitive in self.document['meshes'][0]['primitives']:
            positions = accessor_values(self.document, self.binary, primitive['attributes']['POSITION'])
            normals = accessor_values(self.document, self.binary, primitive['attributes']['NORMAL'])
            uv = accessor_values(self.document, self.binary, primitive['attributes']['TEXCOORD_0'])
            indices = accessor_values(self.document, self.binary, primitive['indices']).ravel()
            self.assertTrue(np.isfinite(positions).all())
            self.assertTrue(np.isfinite(normals).all())
            self.assertTrue(np.isfinite(uv).all())
            self.assertEqual(len(indices) % 3, 0)
            self.assertLess(int(indices.max()), len(positions))
            self.assertEqual(len(normals), len(positions))
            triangles = positions[indices.reshape(-1, 3)]
            geometric = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
            self.assertTrue((np.linalg.norm(geometric, axis=1) > 1e-10).all())
            averaged = normals[indices.reshape(-1, 3)].mean(axis=1)
            self.assertTrue((np.einsum('ij,ij->i', geometric, averaged) > 0).all())
            np.testing.assert_allclose(np.linalg.norm(normals, axis=1), 1.0, atol=1e-6)
            triangle_count += len(indices) // 3
        self.assertEqual(triangle_count, self.metadata['triangle_count'])
        self.assertGreater(triangle_count, 1000)

    def test_materials_embedded_pngs_and_accessor_bounds_match_the_geometry(self):
        document = self.document
        self.assertEqual(len(document['meshes']), 1)
        names = {material['name'] for material in document['materials']}
        self.assertTrue({'Concrete_WarmGrey', 'Coated_Steel', 'Pipe_Insulation',
                         'Safety_Marking', 'Light_Cover'} <= names)
        used = {document['materials'][p['material']]['name']
                for p in document['meshes'][0]['primitives']}
        self.assertTrue({'Concrete_WarmGrey', 'Coated_Steel', 'Pipe_Insulation',
                         'Light_Cover'} <= used)
        for image in document['images']:
            self.assertEqual(image['mimeType'], 'image/png')
            view = document['bufferViews'][image['bufferView']]
            payload = self.binary[view['byteOffset']:view['byteOffset'] + view['byteLength']]
            self.assertTrue(payload.startswith(b'\x89PNG\r\n\x1a\n'))
            self.assertIsNotNone(cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_UNCHANGED))
        concrete = next(m for m in document['materials'] if m['name'] == 'Concrete_WarmGrey')
        self.assertEqual(concrete['pbrMetallicRoughness']['baseColorTexture']['index'], 0)
        self.assertEqual(document['textures'][0]['source'], 0)
        all_positions = []
        for primitive in document['meshes'][0]['primitives']:
            index = primitive['attributes']['POSITION']
            values = accessor_values(document, self.binary, index)
            np.testing.assert_allclose(values.min(axis=0), document['accessors'][index]['min'], atol=1e-5)
            np.testing.assert_allclose(values.max(axis=0), document['accessors'][index]['max'], atol=1e-5)
            all_positions.append(values)
        positions = np.concatenate(all_positions)
        bounds = self.metadata['bounds_m']
        np.testing.assert_allclose(positions.min(axis=0),
                                   [bounds['min'][0], bounds['min'][2], -bounds['max'][1]], atol=0.001)
        np.testing.assert_allclose(positions.max(axis=0),
                                   [bounds['max'][0], bounds['max'][2], -bounds['min'][1]], atol=0.001)
        self.assertGreater(bounds['max'][0] - bounds['min'][0], 70)

    def test_infrastructure_and_four_loop_topology_are_retained(self):
        counts = self.metadata['component_counts']
        for component in ('corridor_shell', 'junction_shell', 'structural_joint', 'pipe_run',
                          'cable_tray', 'support_bracket', 'drain_channel', 'luminaire',
                          'portal_structural_return', 'junction_pipe_run', 'junction_port_coupling'):
            self.assertGreater(counts[component], 0)
        self.assertEqual(self.metadata['layout_summary'],
                         {'node_count': 9, 'edge_count': 12, 'branch_count': 2, 'cycle_rank': 4})
        for fixture in ('structural_joint', 'luminaire'):
            self.assertGreater(len(set(self.metadata['edge_fixture_spacing_m'][fixture].values())), 1)
        routing = self.metadata['service_routing']
        self.assertEqual(routing['unconnected_pipe_ends'], 0)
        # Twelve shared corridors and two dead ends make 26 junction ports.
        self.assertEqual(routing['junction_service_return_count'], 26)
        self.assertEqual(routing['junction_port_coupling_count'], 104)
        self.assertEqual(routing['branch_lane_transition_count'], 2)

    def test_service_fixtures_preserve_the_clear_flight_envelope(self):
        clearance = self.metadata['clearance']
        self.assertEqual(clearance['violations'], [])
        self.assertGreater(clearance['fixture_count_checked'], 0)
        self.assertGreaterEqual(clearance['minimum_lateral_margin_m'], 0.05)
        self.assertGreaterEqual(clearance['minimum_vertical_margin_m'], 0.05)

    def test_service_returns_leave_no_visible_center_crossbars(self):
        junctions = self.metadata['service_routing']['junctions']
        self.assertEqual(len(junctions), 9)
        self.assertTrue(all(j['visible_center_crossbars'] == 0 for j in junctions))


class CorridorTerminationTests(unittest.TestCase):
    def test_cable_tray_rails_and_rungs_are_checked_against_the_flight_envelope(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        from generate_utility_gallery_glb import _SceneMesh, _add_corridor_services
        scene = _SceneMesh()
        _add_corridor_services(
            scene, 'NarrowBranch_Test', (0.0, 0.0), (10.0, 0.0), 3.0, 3.3,
            {'width': 2.2, 'height': 2.2, 'center_height': 1.8},
            {'structural_joint_spacing_m': 5.4, 'cable_support_spacing_m': 2.7,
             'light_spacing_m': 7.0})
        tray_records = [record for record in scene.clearance_records
                        if record['fixture'].startswith('cable_tray_')]
        tray_violations = [violation for violation in scene.clearance_violations
                           if violation['fixture'].startswith('cable_tray_')]
        self.assertGreaterEqual(len(tray_records), 25)
        self.assertGreaterEqual(len(tray_violations), 2)

    def test_dead_end_pipes_terminate_inside_the_25cm_bulkhead(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        from generate_utility_gallery_glb import _SceneMesh, _add_corridor_services
        scene = _SceneMesh()
        _add_corridor_services(
            scene, 'ServiceSpur_Test', (0.0, 0.0), (10.0, 0.0), 3.0, 3.3,
            {'width': 2.0, 'height': 2.2, 'center_height': 1.8},
            {'structural_joint_spacing_m': 5.4, 'cable_support_spacing_m': 2.7,
             'light_spacing_m': 7.0}, terminate_at_end=True)
        pipe_end = max(p[0] for p in scene.groups['Pipe_WaterBlue']['positions'])
        self.assertGreater(pipe_end, 10.0)
        self.assertLess(pipe_end, 10.25)
        self.assertEqual(scene.service_routing['branch_wall_penetration_count'], 4)


class MaterialTextureTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        from generate_utility_gallery_glb import _concrete_material_textures
        self.textures = _concrete_material_textures

    def test_procedural_maps_are_reproducible_without_files_or_platform_fonts(self):
        first, second = self.textures(), self.textures()
        self.assertEqual(first, second)
        maps = [cv2.imdecode(np.frombuffer(p, np.uint8), cv2.IMREAD_UNCHANGED) for p in first]
        self.assertEqual(maps[0].shape, (128, 128, 3))
        self.assertGreater(np.ptp(maps[0]), 0)
        self.assertTrue((maps[2][:, :, 1] >= 225).all())
        self.assertTrue((maps[2][:, :, 0] == 0).all())

    def test_explicit_external_maps_use_original_crop_and_gltf_roughness_channel(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            images = (np.full((700, 900, 3), [40, 80, 120], np.uint8),
                      np.full((700, 900, 3), [255, 128, 128], np.uint8),
                      np.full((700, 900), 10, np.uint8))
            names = ('shaft_albedo.png', 'shaft_normal.png', 'shaft_roughness.png')
            for name, value in zip(names, images):
                success, payload = cv2.imencode('.png', value)
                self.assertTrue(success)
                (root / name).write_bytes(payload.tobytes())
            maps = [cv2.imdecode(np.frombuffer(p, np.uint8), cv2.IMREAD_UNCHANGED)
                    for p in self.textures(root)]
            self.assertEqual(maps[0].shape, (512, 512, 3))
            self.assertEqual(maps[0][0, 0].tolist(), [40, 80, 120])
            self.assertEqual(maps[1][0, 0].tolist(), [255, 128, 128])
            self.assertEqual(maps[2][0, 0].tolist(), [0, 225, 0, 255])

    def test_missing_and_small_external_maps_do_not_fall_back_silently(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with self.assertRaises(FileNotFoundError):
                self.textures(root)
            success, payload = cv2.imencode('.png', np.zeros((16, 16, 3), np.uint8))
            self.assertTrue(success)
            for name in ('shaft_albedo.png', 'shaft_normal.png', 'shaft_roughness.png'):
                (root / name).write_bytes(payload.tobytes())
            with self.assertRaises(ValueError):
                self.textures(root)


if __name__ == '__main__':
    unittest.main()
