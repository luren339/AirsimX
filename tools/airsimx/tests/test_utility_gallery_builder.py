"""Exercise preview inputs, protected-map hashes and offline CLI plans."""

import hashlib
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace
from unittest import mock

from _generated_geometry_test_utils import TOOLS
from _ue_asset_test_utils import import_engine


class PreviewWorkflowTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.project = self.root / 'Project With Spaces'
        self.map_file = self.project / 'Content/Existing/Original.umap'
        self.map_file.parent.mkdir(parents=True)
        self.map_file.write_bytes(b'existing level must not change')
        self.glb = self.root / 'generated.glb'
        self.glb.write_bytes(b'glTF synthetic input for plan-only tests')
        self.geometry = self.root / 'geometry.json'
        self.geometry_data = {
            'status': 'utility_gallery_geometry_generated',
            'glb_sha256': hashlib.sha256(self.glb.read_bytes()).hexdigest(),
            'layout_summary': {'cycle_rank': 4}, 'clearance': {'violations': []},
            'lights': [], 'player_start_m': [-28, -22, 1.25],
            'bounds_m': {'min': [-5, -7, -0.25], 'max': [9, 11, 3.85]},
            'material_names': ['Concrete', 'Steel'],
        }
        self.geometry.write_text(json.dumps(self.geometry_data))
        self.report = self.root / 'report.json'

    def builder(self, *extra):
        return subprocess.run(
            [sys.executable, str(TOOLS / 'scenes/build_utility_gallery_preview.py'),
             '--project-dir', str(self.project), '--glb', str(self.glb),
             '--metadata', str(self.geometry), '--report', str(self.report), '--dry-run', *extra],
            cwd=self.root, capture_output=True, text=True)

    def test_builder_plan_hashes_existing_maps_and_uses_custom_game_paths(self):
        before = self.map_file.read_bytes()
        result = self.builder('--map-asset', '/Game/Independent/Maps/Gallery',
                              '--asset-root', '/Game/Independent/Generated')
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(self.report.read_text())
        self.assertEqual(report['status'], 'utility_gallery_preview_planned')
        self.assertEqual(report['map_asset'], '/Game/Independent/Maps/Gallery')
        self.assertEqual(report['asset_root'], '/Game/Independent/Generated')
        self.assertEqual(report['map_file'],
                         str((self.project / 'Content/Independent/Maps/Gallery.umap').resolve()))
        self.assertEqual(report['existing_map_sha256_before'],
                         {'Content/Existing/Original.umap': hashlib.sha256(before).hexdigest()})
        self.assertEqual(self.map_file.read_bytes(), before)
        self.assertFalse(Path(report['map_file']).exists())
        self.assertFalse(report['engine_executed'])

    def test_builder_rejects_invalid_geometry_before_any_project_write(self):
        for case in ('topology', 'clearance', 'hash'):
            with self.subTest(case=case):
                data = json.loads(json.dumps(self.geometry_data))
                if case == 'topology':
                    data['layout_summary']['cycle_rank'] = 3
                elif case == 'clearance':
                    data['clearance']['violations'] = [{'fixture': 'blocked'}]
                else:
                    data['glb_sha256'] = '0' * 64
                self.geometry.write_text(json.dumps(data))
                result = self.builder()
                self.assertNotEqual(result.returncode, 0)
                self.assertEqual(list(self.project.rglob('*.umap')), [self.map_file])
                self.assertFalse(self.report.exists())

    def test_builder_rejects_paths_that_escape_game_content(self):
        for option, value in (('--map-asset', '/Game/../Existing'),
                               ('--asset-root', '/Engine/Generated')):
            with self.subTest(option=option):
                result = self.builder(option, value)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.report.exists())

    def test_builder_report_cannot_overwrite_unreal_assets_or_inputs(self):
        protected = []
        for suffix in ('.umap', '.uasset'):
            path = self.project / 'Content/Protected' / f'Original{suffix}'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'protected Unreal asset bytes')
            protected.append(path)
        protected.extend((self.glb, self.geometry))
        project_descriptor = self.project / 'Gallery.uproject'
        project_descriptor.write_bytes(b'Unreal project descriptor bytes')
        protected.append(project_descriptor)
        for path in protected:
            with self.subTest(path=path):
                before = path.read_bytes()
                result = self.builder('--report', str(path))
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(path.read_bytes(), before)

    def test_invalid_light_intensity_is_rejected_before_a_plan_is_written(self):
        for intensity in ('0', 'nan'):
            with self.subTest(intensity=intensity):
                result = self.builder('--light-intensity', intensity)
                self.assertNotEqual(result.returncode, 0)
                self.assertFalse(self.report.exists())

    def test_validator_plan_detects_changed_protected_map(self):
        build = self.root / 'build.json'
        target = self.project / 'Content/Independent/Maps/Gallery.umap'
        target.parent.mkdir(parents=True)
        target.write_bytes(b'pretend saved map; no engine is exercised')
        build.write_text(json.dumps({
            'status': 'utility_gallery_preview_level_saved',
            'map_asset': '/Game/Independent/Maps/Gallery',
            'glb_source': str(self.glb), 'geometry_metadata': str(self.geometry),
            'glb_sha256': self.geometry_data['glb_sha256'],
            'existing_map_sha256_before': {'Content/Existing/Original.umap':
                                          hashlib.sha256(self.map_file.read_bytes()).hexdigest()},
        }))
        command = [sys.executable, str(TOOLS / 'scenes/validate_utility_gallery_preview.py'),
                   '--project-dir', str(self.project), '--build-report', str(build),
                   '--map-asset', '/Game/Independent/Maps/Gallery',
                   '--report', str(self.report), '--dry-run']
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        report = json.loads(self.report.read_text())
        self.assertEqual(report['status'], 'utility_gallery_validation_planned')
        self.assertFalse(report['engine_executed'])
        self.report.unlink()
        self.map_file.write_bytes(b'changed')
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse(self.report.exists())

    def test_validator_report_cannot_overwrite_umap_uasset_or_build_report(self):
        target = self.project / 'Content/Independent/Maps/Gallery.umap'
        target.parent.mkdir(parents=True)
        target.write_bytes(b'preview map bytes')
        build = self.root / 'build.json'
        build.write_text(json.dumps({
            'status': 'utility_gallery_preview_level_saved',
            'map_asset': '/Game/Independent/Maps/Gallery',
            'glb_source': str(self.glb), 'geometry_metadata': str(self.geometry),
            'glb_sha256': self.geometry_data['glb_sha256'],
            'existing_map_sha256_before': {'Content/Existing/Original.umap':
                                          hashlib.sha256(self.map_file.read_bytes()).hexdigest()},
        }))
        protected = [build, self.glb, self.geometry]
        project_descriptor = self.project / 'Gallery.uproject'
        project_descriptor.write_bytes(b'Unreal project descriptor bytes')
        protected.append(project_descriptor)
        for suffix in ('.umap', '.uasset'):
            path = self.project / 'Content/Protected' / f'Original{suffix}'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(b'protected Unreal asset bytes')
            protected.append(path)
        command = [sys.executable, str(TOOLS / 'scenes/validate_utility_gallery_preview.py'),
                   '--project-dir', str(self.project), '--build-report', str(build),
                   '--map-asset', '/Game/Independent/Maps/Gallery', '--dry-run']
        for path in protected:
            with self.subTest(path=path):
                before = path.read_bytes()
                result = subprocess.run([*command, '--report', str(path)],
                                        cwd=self.root, capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(path.read_bytes(), before)


class CollisionConfigurationTests(unittest.TestCase):
    """Controlled engine boundary; actual UE execution is reported separately."""

    def setUp(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        import build_utility_gallery_preview
        self.builder = build_utility_gallery_preview

    def engine(self, *, available=True, removal=True, simple=0, convex=0):
        class Subsystem:
            def remove_collisions(self, mesh):
                mesh.simple, mesh.convex = simple, convex
                return removal

            def get_simple_collision_count(self, mesh):
                return mesh.simple

            def get_convex_collision_count(self, mesh):
                return mesh.convex

        mesh = SimpleNamespace(simple=2, convex=4, saved=False, body={})
        body = SimpleNamespace(set_editor_property=lambda key, value: mesh.body.update({key: value}))
        mesh.get_editor_property = lambda key: body
        def save(path):
            mesh.saved = True
            return True
        subsystem = Subsystem()
        api = SimpleNamespace(
            StaticMeshEditorSubsystem=object(),
            get_editor_subsystem=lambda kind: subsystem if available else None,
            # Reproduce deprecated wrapper fallback when no subsystem exists.
            EditorStaticMeshLibrary=subsystem,
            EditorAssetLibrary=SimpleNamespace(save_asset=save),
            CollisionTraceFlag=SimpleNamespace(CTF_USE_COMPLEX_AS_SIMPLE='complex'),
        )
        return api, mesh

    def test_collision_removes_convex_hulls_and_saves_complex_only_mode(self):
        api, mesh = self.engine()
        with mock.patch.object(self.builder, 'unreal', api):
            report = self.builder.configure_mesh_collision('/Game/Generated/Mesh', mesh)
        self.assertTrue(mesh.saved)
        self.assertEqual(mesh.body['collision_trace_flag'], 'complex')
        self.assertEqual(report['remaining_simple_collision_count'], 0)
        self.assertEqual(report['remaining_convex_collision_count'], 0)

    def test_missing_editor_subsystem_reports_how_to_run_the_editor(self):
        api, mesh = self.engine(available=False, simple=-1)
        with mock.patch.object(self.builder, 'unreal', api):
            with self.assertRaisesRegex(RuntimeError, 'StaticMeshEditorSubsystem.*ExecutePythonScript'):
                self.builder.configure_mesh_collision('/Game/Generated/Mesh', mesh)
        self.assertFalse(mesh.saved)

    def test_failed_removal_invalid_query_and_remaining_convex_hulls_are_rejected(self):
        for values in ({'removal': False}, {'simple': -1}, {'convex': 1}):
            with self.subTest(values=values):
                api, mesh = self.engine(**values)
                with mock.patch.object(self.builder, 'unreal', api):
                    with self.assertRaises(RuntimeError):
                        self.builder.configure_mesh_collision('/Game/Generated/Mesh', mesh)
                self.assertFalse(mesh.saved)


class LevelSafetyTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        import _preview_workflow as workflow
        import build_utility_gallery_preview as builder
        self.workflow = workflow
        self.builder = builder

    @staticmethod
    def fake_actor(package_name, label='UtilityGallery_MainStructure'):
        package = SimpleNamespace(get_name=lambda: package_name)
        level = SimpleNamespace(get_outermost=lambda: package)
        return SimpleNamespace(get_level=lambda: level, get_actor_label=lambda: label)

    def fake_unreal(self, *, dirty=True, exists=True, actors=()):
        package = SimpleNamespace(get_name=lambda: '/Game/Current/Unsaved')
        persistent_level = SimpleNamespace(get_outermost=lambda: package)
        world = SimpleNamespace(get_persistent_level=lambda: persistent_level)
        level_library = SimpleNamespace(
            get_editor_world=lambda: world,
            load_level=mock.Mock(return_value=True),
            new_level=mock.Mock(return_value=True),
            get_all_level_actors=lambda: list(actors),
            destroy_actor=mock.Mock(),
        )
        api = SimpleNamespace(
            EditorLoadingAndSavingUtils=SimpleNamespace(
                get_dirty_map_packages=lambda: [package] if dirty else []),
            EditorLevelLibrary=level_library,
            EditorAssetLibrary=SimpleNamespace(does_asset_exist=lambda _: exists),
        )
        return api

    def test_dirty_persistent_level_blocks_both_load_and_create(self):
        for exists in (True, False):
            with self.subTest(existing_target=exists):
                api = self.fake_unreal(dirty=True, exists=exists)
                with mock.patch.object(self.builder, 'unreal', api):
                    with self.assertRaisesRegex(RuntimeError, 'unsaved changes'):
                        self.builder.load_or_create_level(
                            SimpleNamespace(map_asset='/Game/Generated/Preview'))
                api.EditorLevelLibrary.load_level.assert_not_called()
                api.EditorLevelLibrary.new_level.assert_not_called()

    def test_dirty_streaming_sublevel_blocks_load_and_create_with_clean_persistent_level(self):
        streaming_package = SimpleNamespace(get_name=lambda: '/Game/Current/DirtySublevel')
        for exists in (True, False):
            with self.subTest(existing_target=exists):
                api = self.fake_unreal(dirty=False, exists=exists)
                # Unreal's Python World wrapper does not expose get_persistent_level().
                api.EditorLevelLibrary.get_editor_world = lambda: SimpleNamespace()
                api.EditorLoadingAndSavingUtils.get_dirty_map_packages = lambda: [streaming_package]
                api.EditorLoadingAndSavingUtils.save_dirty_packages = mock.Mock()
                with mock.patch.object(self.builder, 'unreal', api):
                    with self.assertRaisesRegex(RuntimeError, 'unsaved changes'):
                        self.builder.load_or_create_level(
                            SimpleNamespace(map_asset='/Game/Generated/Preview'))
                api.EditorLevelLibrary.load_level.assert_not_called()
                api.EditorLevelLibrary.new_level.assert_not_called()
                api.EditorLoadingAndSavingUtils.save_dirty_packages.assert_not_called()

    def test_only_actors_owned_by_the_target_level_are_selected(self):
        target = self.fake_actor('/Game/Generated/Preview')
        sublevel = self.fake_actor('/Game/Shared/Sublevel')
        unlabeled = SimpleNamespace(get_level=lambda: None)
        selected = self.workflow.actors_in_map(
            [target, sublevel, unlabeled], '/Game/Generated/Preview')
        self.assertEqual(selected, [target])

    def test_builder_deletes_generated_actors_only_in_target_level(self):
        target_actor = self.fake_actor('/Game/Generated/Preview')
        other_level_actor = self.fake_actor('/Game/Shared/Sublevel')
        api = self.fake_unreal(dirty=False, exists=True,
                               actors=[target_actor, other_level_actor])
        with mock.patch.object(self.builder, 'unreal', api):
            self.builder.load_or_create_level(
                SimpleNamespace(map_asset='/Game/Generated/Preview'))
        api.EditorLevelLibrary.destroy_actor.assert_called_once_with(target_actor)


class DirtyValidatorLevelTests(unittest.TestCase):
    def test_unsaved_level_is_rejected_before_validator_loads_the_map(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        import validate_utility_gallery_preview as validator
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            project = root / 'Project'
            project.mkdir()
            build_report = root / 'build.json'
            build_report.write_text(json.dumps({
                'map_asset': '/Game/Generated/Preview',
                'glb_source': str(root / 'scene.glb'),
                'geometry_metadata': str(root / 'geometry.json'),
            }))
            current_package = SimpleNamespace(get_name=lambda: '/Game/Current/Unsaved')
            persistent_level = SimpleNamespace(get_outermost=lambda: current_package)
            world = SimpleNamespace(get_persistent_level=lambda: persistent_level)
            load_level = mock.Mock(return_value=True)
            api = SimpleNamespace(
                EditorLevelLibrary=SimpleNamespace(get_editor_world=lambda: world,
                                                   load_level=load_level),
                EditorLoadingAndSavingUtils=SimpleNamespace(
                    get_dirty_map_packages=lambda: [current_package]),
            )
            args = SimpleNamespace(project_dir=project, build_report=build_report,
                                   report=root / 'validation.json',
                                   map_asset='/Game/Generated/Preview')
            with mock.patch.object(validator.importlib, 'import_module', return_value=api), \
                    mock.patch.object(validator, 'check_project_dir'), \
                    mock.patch.object(validator, 'read_geometry', return_value={'glb_sha256': 'hash'}):
                with self.assertRaisesRegex(RuntimeError, 'unsaved changes'):
                    validator.run_validation(args)
            load_level.assert_not_called()

    def test_dirty_streaming_sublevel_is_rejected_before_validator_loads_target(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        import validate_utility_gallery_preview as validator
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            project = root / 'Project'
            project.mkdir()
            build_report = root / 'build.json'
            build_report.write_text(json.dumps({
                'map_asset': '/Game/Generated/Preview',
                'glb_source': str(root / 'scene.glb'),
                'geometry_metadata': str(root / 'geometry.json'),
            }))
            streaming_package = SimpleNamespace(get_name=lambda: '/Game/Current/DirtySublevel')
            # Unreal's Python World wrapper does not expose get_persistent_level().
            world = SimpleNamespace()
            load_level = mock.Mock(side_effect=RuntimeError('level switch was called'))
            save_dirty_packages = mock.Mock()
            api = SimpleNamespace(
                EditorLevelLibrary=SimpleNamespace(get_editor_world=lambda: world,
                                                   load_level=load_level),
                EditorLoadingAndSavingUtils=SimpleNamespace(
                    get_dirty_map_packages=lambda: [streaming_package],
                    save_dirty_packages=save_dirty_packages),
            )
            args = SimpleNamespace(project_dir=project, build_report=build_report,
                                   report=root / 'validation.json',
                                   map_asset='/Game/Generated/Preview')
            with mock.patch.object(validator.importlib, 'import_module', return_value=api), \
                    mock.patch.object(validator, 'check_project_dir'), \
                    mock.patch.object(validator, 'read_geometry', return_value={'glb_sha256': 'hash'}):
                with self.assertRaisesRegex(RuntimeError, 'unsaved changes'):
                    validator.run_validation(args)
            load_level.assert_not_called()
            save_dirty_packages.assert_not_called()


class ValidatorTargetLevelTests(unittest.TestCase):
    def test_same_labels_in_shared_sublevel_do_not_change_preview_actor_counts(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        import validate_utility_gallery_preview as validator
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory).resolve()
            project = root / 'Project'
            project.mkdir()
            build_report = root / 'build.json'
            build_report.write_text(json.dumps({
                'map_asset': '/Game/Generated/Preview',
                'glb_source': str(root / 'scene.glb'),
                'geometry_metadata': str(root / 'geometry.json'),
                'point_light_count': 0,
                'loop_summary': {'cycle_rank': 4},
                'clearance': {'violations': []},
                'existing_map_sha256_before': {},
                'static_mesh_asset': '/Game/Generated/Gallery.Gallery',
                'static_mesh_material_count': 2,
                'actor_bounds_origin_cm': [0.0, 0.0, 0.0],
                'actor_bounds_extent_cm': [1.0, 1.0, 1.0],
                'actor_location_cm': [0.0, 0.0, 0.0],
                'actor_rotation_rpy_deg': [0.0, 0.0, 0.0],
                'actor_scale': [1.0, 1.0, 1.0],
            }))
            def actor(label, package_name):
                package = SimpleNamespace(get_name=lambda: package_name)
                level = SimpleNamespace(get_outermost=lambda: package)
                return SimpleNamespace(get_level=lambda: level,
                                       get_actor_label=lambda: label)

            component = SimpleNamespace(
                get_editor_property=lambda _: SimpleNamespace(),
                get_collision_enabled=lambda: 'QUERY_AND_PHYSICS',
            )
            body = SimpleNamespace(get_editor_property=lambda _: 'CTF_USE_COMPLEX_AS_SIMPLE')
            mesh = SimpleNamespace(
                get_editor_property=lambda key: body if key == 'body_setup' else [1, 2],
                get_path_name=lambda: '/Game/Generated/Gallery.Gallery',
            )
            component.get_editor_property = lambda _: mesh
            structure = actor('UtilityGallery_MainStructure', '/Game/Generated/Preview')
            structure.get_component_by_class = lambda _: component
            structure.get_actor_bounds = lambda _: (
                SimpleNamespace(x=0.0, y=0.0, z=0.0),
                SimpleNamespace(x=1.0, y=1.0, z=1.0),
            )
            sublevel_structure = actor('UtilityGallery_MainStructure', '/Game/Shared/Sublevel')
            sublevel_light = actor('UtilityGallery_Light_001', '/Game/Shared/Sublevel')
            player_start_type = type('PlayerStart', (), {})
            target_player_start = player_start_type()
            target_player_start.get_actor_label = lambda: 'UtilityGallery_PlayerStart'
            target_player_start.get_level = lambda: actor(
                'owner', '/Game/Generated/Preview').get_level()
            sublevel_player_start = player_start_type()
            sublevel_player_start.get_actor_label = lambda: 'UtilityGallery_PlayerStart'
            sublevel_player_start.get_level = lambda: actor(
                'owner', '/Game/Shared/Sublevel').get_level()
            world_package = SimpleNamespace(get_name=lambda: '/Game/Other/Current')
            level = SimpleNamespace(get_outermost=lambda: world_package)
            world = SimpleNamespace(get_persistent_level=lambda: level)
            api = SimpleNamespace(
                EditorLevelLibrary=SimpleNamespace(
                    get_editor_world=lambda: world,
                    load_level=lambda _: True,
                    get_all_level_actors=lambda: [structure, sublevel_structure, sublevel_light,
                                                  target_player_start, sublevel_player_start],
                ),
                EditorLoadingAndSavingUtils=SimpleNamespace(get_dirty_map_packages=lambda: []),
                PlayerStart=player_start_type,
                StaticMeshComponent=type('StaticMeshComponent', (), {}),
                SystemLibrary=SimpleNamespace(get_engine_version=lambda: 'controlled-boundary'),
            )
            args = SimpleNamespace(project_dir=project, build_report=build_report,
                                   report=root / 'validation.json',
                                   map_asset='/Game/Generated/Preview')
            with mock.patch.object(validator.importlib, 'import_module', return_value=api), \
                    mock.patch.object(validator, 'check_project_dir'), \
                    mock.patch.object(validator, 'read_geometry', return_value={}), \
                    mock.patch.object(validator, 'validate_scene_observation'), \
                    mock.patch.object(validator, 'static_mesh_editor', return_value=object()), \
                    mock.patch.object(validator, 'collision_counts', return_value=(0, 0)), \
                    mock.patch.object(validator, 'hash_existing_maps', return_value={}), \
                    mock.patch.object(validator, 'actor_transform', return_value={}):
                report = validator.run_validation(args)
            self.assertEqual(report['gallery_structure_actor_count'], 1)
            self.assertEqual(report['point_light_count'], 0)
            self.assertEqual(report['player_start_count'], 1)


class GalleryImportResultTests(unittest.TestCase):
    def test_empty_task_result_never_falls_back_to_an_existing_directory_mesh(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        import build_utility_gallery_preview as builder
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            glb, metadata = root / 'scene.glb', root / 'geometry.json'
            glb.write_bytes(b'glTF')
            metadata.write_text('{}')
            args = SimpleNamespace(glb=glb, metadata=metadata,
                                   asset_root='/Game/Old', replace_existing=True)
            api, state = import_engine([])
            with mock.patch.object(builder, 'unreal', api):
                with self.assertRaises(RuntimeError):
                    builder.import_gallery_mesh(args)
            self.assertEqual(state.loads, [])


class SceneReopenContractTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, str(TOOLS / 'scenes'))
        import _preview_workflow as workflow
        self.workflow = workflow
        self.geometry = {'bounds_m': {'min': [-5, -7, -0.25], 'max': [9, 11, 3.85]},
                         'material_names': ['Concrete', 'Steel']}
        self.build = {'static_mesh_asset': '/Game/Generated/Small.Small',
                      'static_mesh_material_count': 2,
                      'actor_bounds_origin_cm': [200, -200, 180],
                      'actor_bounds_extent_cm': [700, 900, 205],
                      'actor_location_cm': [0, 0, 0], 'actor_rotation_rpy_deg': [0, 0, 0],
                      'actor_scale': [1, 1, 1]}

    def test_compact_scene_is_checked_against_transformed_manifest_bounds(self):
        self.workflow.validate_scene_observation(self.build, self.geometry, copy.deepcopy(self.build))

    def test_reopen_rejects_other_assets_dimensions_materials_and_actor_transforms(self):
        changes = {'static_mesh_asset': '/Game/Other/Large.Large',
                   'static_mesh_material_count': 3,
                   'actor_bounds_origin_cm': [201, -200, 180],
                   'actor_bounds_extent_cm': [4325, 2625, 205],
                   'actor_location_cm': [1, 0, 0], 'actor_rotation_rpy_deg': [0, 0, 1],
                   'actor_scale': [1.1, 1, 1]}
        for field, value in changes.items():
            with self.subTest(field=field):
                observation = copy.deepcopy(self.build)
                observation[field] = value
                with self.assertRaises(RuntimeError):
                    self.workflow.validate_scene_observation(self.build, self.geometry, observation)

    def test_build_bounds_must_also_match_the_manifest(self):
        self.build['actor_bounds_extent_cm'] = [4325, 2625, 205]
        with self.assertRaises(RuntimeError):
            self.workflow.validate_scene_observation(self.build, self.geometry, copy.deepcopy(self.build))

    def test_submillimeter_import_rounding_is_allowed(self):
        observation = copy.deepcopy(self.build)
        observation['actor_bounds_origin_cm'][0] += 0.01
        self.workflow.validate_scene_observation(self.build, self.geometry, observation)


if __name__ == '__main__':
    unittest.main()
