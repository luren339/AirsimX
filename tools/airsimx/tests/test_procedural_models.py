"""Real OBJ output, bounds and optional external placement-data conversion."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

from _generated_geometry_test_utils import TOOLS, read_obj


class ModelGenerationTests(unittest.TestCase):
    def test_model_clis_generate_finite_geometry_with_measured_bounds(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'generated'
            for script, target in (('generate_open_quadrotor_frame.py', output / 'frame'),
                                   ('generate_sensor_rig_meshes.py', output)):
                result = subprocess.run(
                    [sys.executable, str(TOOLS / 'models' / script), '--output-dir', str(target)],
                    cwd=directory, capture_output=True, text=True)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            cases = (
                ('frame/OpenQuadrotorFrame30cm.obj', [30, 22.2, 9.27],
                 'frame/open_quadrotor_frame_manifest.json'),
                ('camera/approximate_camera_shell.obj', [4.005, 2.9, 2.18],
                 'camera/approximate_model_manifest.json'),
                ('lidar_mount/mid360s_mount_plate.obj', [13.7, 10, 1.67],
                 'lidar_mount/mount_manifest.json'),
            )
            for relative, expected, manifest_name in cases:
                with self.subTest(model=relative):
                    path = output / relative
                    vertices, faces = read_obj(path)
                    self.assertTrue(np.isfinite(vertices).all())
                    self.assertGreater(len(vertices), 20)
                    self.assertGreater(len(faces), 10)
                    np.testing.assert_allclose(np.ptp(vertices, axis=0), expected, atol=1e-6)
                    for face in faces:
                        self.assertGreaterEqual(min(face), 0)
                        self.assertLess(max(face), len(vertices))
                        self.assertGreaterEqual(len(face), 3)
                        points = vertices[face]
                        # OBJ n-gons are fan-triangulable; reject zero-area faces.
                        for i in range(1, len(points) - 1):
                            self.assertGreater(np.linalg.norm(np.cross(
                                points[i] - points[0], points[i + 1] - points[0])), 1e-10)
                    manifest = json.loads((output / manifest_name).read_text())
                    np.testing.assert_allclose(manifest['geometry']['size_cm'], expected, atol=1e-6)
                    self.assertEqual(manifest['geometry']['vertex_count'], len(vertices))
                    self.assertTrue(path.with_suffix('.mtl').is_file())
                    self.assertIn('visual', manifest['note'].lower())


class MeshPlacementDataTests(unittest.TestCase):
    def instances(self):
        return {'mesh_actor_instances': [
            {'static_mesh_asset': '/Game/Example/Part_%02d.Part_%02d' % (i, i),
             'translation': [i, -2, 3], 'rotation_xyzw': [0, 0, 0, 1], 'scale': [1, 1, 1]}
            for i in range(13)]}

    def run_converter(self, directory, data):
        source = Path(directory) / 'external_transforms.json'
        output = Path(directory) / 'output/UAVSensorRigMeshData.h'
        source.write_text(json.dumps(data))
        result = subprocess.run(
            [sys.executable, str(TOOLS / 'models/generate_uav_sensor_rig_mesh_data.py'),
             str(source), '--output', str(output)], cwd=directory, capture_output=True, text=True)
        return result, output

    def test_explicit_external_transforms_produce_the_original_cpp_placement_schema(self):
        with tempfile.TemporaryDirectory() as directory:
            result, output = self.run_converter(directory, self.instances())
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            header = output.read_text()
            self.assertEqual(header.count('    {TEXT('), 13)
            self.assertIn('FVector(12.000000000f, -2.000000000f, 3.000000000f)', header)
            self.assertIn('FQuat(0.000000000000f, 0.000000000000f, 0.000000000000f, 1.000000000000f)', header)
            self.assertIn('Mid360SPartCount = UE_ARRAY_COUNT(Mid360SParts)', header)

    def test_invalid_count_nonfinite_transform_and_asset_injection_leave_no_header(self):
        for case in ('count', 'nonfinite', 'asset'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                data = self.instances()
                if case == 'count':
                    data['mesh_actor_instances'].pop()
                elif case == 'nonfinite':
                    data['mesh_actor_instances'][0]['translation'][0] = float('nan')
                else:
                    data['mesh_actor_instances'][0]['static_mesh_asset'] = '/Game/Bad"); malicious('
                result, output = self.run_converter(directory, data)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
