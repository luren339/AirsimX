"""Engine-free validation of model import and derived-Pawn plans."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from _generated_geometry_test_utils import TOOLS
from _ue_asset_test_utils import import_engine


class SensorModelWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.generated = self.root / 'models'
        for relative in ('camera/approximate_camera_shell.obj',
                         'lidar_mount/mid360s_mount_plate.obj', 'frame/OpenQuadrotorFrame30cm.obj'):
            path = self.generated / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('# synthetic file used only to plan an import\n')
        self.report = self.root / 'report.json'

    def run_tool(self, name, *arguments):
        return subprocess.run(
            [sys.executable, str(TOOLS / 'models' / name), '--report', str(self.report),
             '--dry-run', *arguments], cwd=self.root, capture_output=True, text=True)

    def test_import_plan_uses_only_three_procedural_models_and_custom_game_root(self):
        result = self.run_tool('import_generated_sensor_meshes.py', '--generated-dir', str(self.generated),
                               '--asset-root', '/Game/Independent/Drone')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        plan = json.loads(self.report.read_text())
        self.assertEqual(plan['status'], 'sensor_model_import_planned')
        self.assertFalse(plan['engine_executed'])
        self.assertFalse(plan['replace_existing'])
        self.assertEqual(len(plan['items']), 3)
        self.assertEqual({i['asset'] for i in plan['items']}, {
            '/Game/Independent/Drone/Camera/approximate_camera_shell',
            '/Game/Independent/Drone/Mounts/mid360s_mount_plate',
            '/Game/Independent/Drone/Models/OpenQuadrotorFrame30cm'})
        self.assertTrue(all(Path(i['source']).is_file() for i in plan['items']))

    def test_missing_model_fails_before_engine_import_or_report(self):
        (self.generated / 'camera/approximate_camera_shell.obj').unlink()
        result = self.run_tool('import_generated_sensor_meshes.py', '--generated-dir', str(self.generated))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.report.exists())

    def test_rig_plan_has_configurable_parent_assets_and_centimeter_transforms(self):
        result = self.run_tool('create_uav_sensor_rig.py', '--asset-root', '/Game/Independent/Drone',
                               '--blueprint-asset', '/Game/Independent/BP_ExampleRig',
                               '--parent-blueprint', '/AirSim/Blueprints/BP_FlyingPawn')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        plan = json.loads(self.report.read_text())
        self.assertEqual(plan['status'], 'sensor_rig_blueprint_planned')
        self.assertFalse(plan['engine_executed'])
        self.assertEqual(plan['blueprint_asset'], '/Game/Independent/BP_ExampleRig')
        self.assertEqual(plan['parent_blueprint'], '/AirSim/Blueprints/BP_FlyingPawn')
        camera = next(p for p in plan['components'] if p['name'] == 'AirsimXCameraShell')
        self.assertEqual(camera['location_cm'], [8.5, 0, 1])
        self.assertEqual(camera['asset'], '/Game/Independent/Drone/Camera/approximate_camera_shell')
        self.assertTrue(all(p['collision'] == 'NoCollision' for p in plan['components']))
        self.assertEqual(len(plan['components']), 3)

    def test_invalid_game_path_or_nonfinite_transform_produces_no_plan(self):
        result = self.run_tool('create_uav_sensor_rig.py', '--blueprint-asset', '/Game/../Original')
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.report.exists())
        config = json.loads((TOOLS / 'models/settings/sensor_rig.json').read_text())
        config['components'][0]['location_cm'][0] = float('nan')
        path = self.root / 'invalid.json'
        path.write_text(json.dumps(config))
        result = self.run_tool('create_uav_sensor_rig.py', '--config', str(path))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.report.exists())


class InheritedRootSelectionTests(unittest.TestCase):
    def test_multiple_root_flags_do_not_override_the_parent_cdo_root(self):
        sys.path.insert(0, str(TOOLS / 'models'))
        import _model_workflow as workflow
        records = [
            {'variable_name': name, 'object_name': name + '_GEN_VARIABLE', 'handle': index,
             'is_scene_component': name == 'BodyMesh', 'is_root_component': True}
            for index, name in enumerate(('Prop0', 'Prop1', 'BodyMesh', 'Prop2', 'Prop3'))
        ]
        self.assertEqual(workflow.select_scene_root(records, 'BodyMesh')['handle'], 2)
        self.assertEqual(workflow.select_scene_root(records, 'BodyMesh_GEN_VARIABLE')['handle'], 2)
        # Blueprint SCS roots can be absent on an uninstantiated parent CDO.
        self.assertEqual(workflow.select_scene_root(records)['handle'], 2)

    def test_missing_or_ambiguous_cdo_root_is_rejected(self):
        sys.path.insert(0, str(TOOLS / 'models'))
        import _model_workflow as workflow
        with self.assertRaises(RuntimeError):
            workflow.select_scene_root([], 'BodyMesh')
        with self.assertRaises(RuntimeError):
            workflow.select_scene_root([
                {'variable_name': 'BodyMesh', 'object_name': 'one', 'is_scene_component': True},
                {'variable_name': 'BodyMesh', 'object_name': 'two', 'is_scene_component': True}], 'BodyMesh')


class SensorImportExecutionTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(TOOLS / 'models'))
        import import_generated_sensor_meshes
        self.importer = import_generated_sensor_meshes
        self.plan = {'replace_existing': False, 'items': [
            {'source': 'camera.obj', 'destination': '/Game/Generated/Camera',
             'asset': '/Game/Generated/Camera/Camera'}]}

    def test_default_import_refuses_a_material_in_the_destination_before_importing(self):
        api, state = import_engine(['/Game/Generated/Camera/Camera.Camera'],
                                   ['/Game/Generated/Camera/housing.housing'])
        with mock.patch.dict(sys.modules, {'unreal': api}):
            with self.assertRaises(RuntimeError):
                self.importer.run_import(self.plan)
        self.assertEqual(state.imports, 0)
        self.assertTrue(state.unrelated_dirty)

    def test_import_never_saves_unrelated_assets_in_the_target_directory(self):
        api, state = import_engine(['/Game/Generated/Camera/Camera.Camera'])
        self.plan['replace_existing'] = True
        with mock.patch.dict(sys.modules, {'unreal': api}):
            report = self.importer.run_import(self.plan)
        self.assertEqual(report['status'], 'sensor_models_imported')
        self.assertTrue(state.unrelated_dirty)

    def test_empty_or_mismatched_task_result_cannot_reuse_an_old_mesh(self):
        self.plan['replace_existing'] = True
        for paths in ([], ['/Game/Generated/Other.Other']):
            with self.subTest(paths=paths):
                api, state = import_engine(paths)
                with mock.patch.dict(sys.modules, {'unreal': api}):
                    with self.assertRaises(RuntimeError):
                        self.importer.run_import(self.plan)
                self.assertEqual(state.loads, [])


if __name__ == '__main__':
    unittest.main()
