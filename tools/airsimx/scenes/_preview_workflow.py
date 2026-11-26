"""Path and input checks shared by the UE preview scripts (standard library only)."""

import hashlib
import json
import math
import os
from pathlib import Path
import re

DEFAULT_OUTPUT = Path(__file__).resolve().parents[3] / 'build/airsimx/scenes'
DEFAULT_MAP = '/Game/AirsimX/UtilityGallery/Maps/UtilityGalleryPreview'
DEFAULT_ASSETS = '/Game/AirsimX/UtilityGallery/Generated'


def game_path(path):
    if not re.fullmatch(r'/Game/(?:[A-Za-z0-9_]+/)*[A-Za-z0-9_]+', path):
        raise ValueError(f'expected a /Game/... package path without extensions or traversal: {path}')
    return path


def map_file(project_dir, map_asset):
    return Path(project_dir).resolve() / 'Content' / (game_path(map_asset)[6:] + '.umap')


def project_descriptor_files(project_dir):
    return tuple(Path(project_dir).resolve().glob('*.uproject'))


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def hash_existing_maps(project_dir, map_asset):
    project_dir = Path(project_dir).resolve()
    target = map_file(project_dir, map_asset)
    return {path.relative_to(project_dir).as_posix(): sha256(path)
            for path in sorted((project_dir / 'Content').rglob('*.umap'))
            if path.is_file() and path.resolve() != target}


def _unreal_object_paths(obj):
    paths = set()
    for getter_name in ('get_path_name', 'get_name'):
        getter = getattr(obj, getter_name, None)
        if getter is not None:
            try:
                value = getter()
            except Exception:
                continue
            if value:
                paths.add(str(value))
    return paths


def assert_current_level_saved(unreal):
    """Refuse a level switch if Unreal reports any dirty map package."""
    level_library = unreal.EditorLevelLibrary
    world = level_library.get_editor_world()
    if world is None:
        raise RuntimeError('cannot inspect the current editor world; refusing to switch levels')
    try:
        dirty_packages = unreal.EditorLoadingAndSavingUtils.get_dirty_map_packages()
    except Exception as error:
        raise RuntimeError('cannot query dirty map packages; refusing to switch levels') from error
    dirty_paths = set()
    unidentified_dirty_package = False
    for package in dirty_packages:
        paths = _unreal_object_paths(package)
        if not paths:
            unidentified_dirty_package = True
        dirty_paths.update(paths)
    if dirty_packages:
        if unidentified_dirty_package:
            raise RuntimeError(
                'one or more dirty map packages could not be identified; '
                'save or close every dirty level in Unreal Editor before building or validating another level')
        raise RuntimeError(
            f'map package(s) {sorted(dirty_paths)} have unsaved changes; '
            'save every dirty level in Unreal Editor before building or validating another level')


def actors_in_map(actors, map_asset):
    """Select actors whose owning level package exactly matches a /Game map."""
    target = game_path(map_asset)
    selected = []
    for actor in actors:
        try:
            level = actor.get_level()
            package = level.get_outermost() if level is not None else None
            package_paths = _unreal_object_paths(package) if package is not None else set()
        except Exception:
            continue
        if target in package_paths:
            selected.append(actor)
    return selected


def validate_report_destination(path, input_paths=()):
    """Reject reports that alias source inputs or Unreal asset packages."""
    target = Path(path).expanduser().resolve(strict=False)
    if target.suffix.lower() in {'.umap', '.uasset'}:
        raise ValueError(f'report path must not target an Unreal asset: {target}')
    for source in input_paths:
        if source is None:
            continue
        source_path = Path(source).expanduser().resolve(strict=False)
        same_file = target == source_path
        if not same_file and target.exists() and source_path.exists():
            try:
                same_file = os.path.samefile(target, source_path)
            except OSError:
                same_file = False
        if same_file:
            raise ValueError(f'report path aliases an input file and would overwrite it: {target}')
    return target


def read_geometry(glb, metadata):
    geometry = json.loads(Path(metadata).read_text(encoding='utf-8'))
    if geometry.get('status') != 'utility_gallery_geometry_generated':
        raise ValueError('expected a generated gallery geometry manifest')
    if geometry['glb_sha256'] != sha256(glb):
        raise ValueError('GLB does not match the geometry manifest SHA-256')
    if geometry['layout_summary']['cycle_rank'] != 4:
        raise ValueError('preview must contain four independent loops')
    if geometry['clearance']['violations']:
        raise ValueError('gallery fixtures intrude into the flight envelope')
    return geometry


def check_project_dir(project_dir, unreal):
    active = Path(unreal.Paths.project_dir()).resolve()
    if active != Path(project_dir).resolve():
        raise RuntimeError(f'active UE project {active} differs from --project-dir {project_dir}')


def write_report(path, report, input_paths=()):
    path = validate_report_destination(path, input_paths)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def metadata_to_ue_cm(position):
    x, y, z = (float(value) * 100 for value in position)
    return [x, -y, z]


def static_mesh_editor(unreal):
    editor = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem)
    if editor is None:
        raise RuntimeError('StaticMeshEditorSubsystem unavailable; run the full editor with -ExecutePythonScript, not -run=pythonscript')
    return editor


def collision_counts(editor, mesh):
    # UE counts convex hulls separately from box/sphere/capsule collisions.
    simple = editor.get_simple_collision_count(mesh)
    convex = editor.get_convex_collision_count(mesh)
    if simple < 0 or convex < 0:
        raise RuntimeError(f'could not query mesh collisions: simple={simple}, convex={convex}')
    return simple, convex


def actor_transform(actor):
    location = actor.get_actor_location()
    rotation = actor.get_actor_rotation()
    scale = actor.get_actor_scale3d()
    return {'actor_location_cm': [float(location.x), float(location.y), float(location.z)],
            'actor_rotation_rpy_deg': [float(rotation.roll), float(rotation.pitch), float(rotation.yaw)],
            'actor_scale': [float(scale.x), float(scale.y), float(scale.z)]}


def _close_vector(actual, expected, tolerance):
    return (len(actual) == len(expected) == 3
            and all(math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tolerance
                    for a, b in zip(actual, expected)))


def validate_scene_observation(build, geometry, observation):
    lower, upper = geometry['bounds_m']['min'], geometry['bounds_m']['max']
    ue_lower = metadata_to_ue_cm([lower[0], upper[1], lower[2]])
    ue_upper = metadata_to_ue_cm([upper[0], lower[1], upper[2]])
    expected = {'actor_bounds_origin_cm': [(a + b) / 2 for a, b in zip(ue_lower, ue_upper)],
                'actor_bounds_extent_cm': [(b - a) / 2 for a, b in zip(ue_lower, ue_upper)]}
    for field, values in expected.items():
        if not _close_vector(build[field], values, 0.1) or not _close_vector(observation[field], values, 0.1):
            raise RuntimeError(f'{field} does not match the geometry manifest (0.1 cm tolerance)')
    if observation['static_mesh_asset'] != build['static_mesh_asset']:
        raise RuntimeError('reopened actor references a different StaticMesh')
    if (observation['static_mesh_material_count'] != build['static_mesh_material_count']
            or observation['static_mesh_material_count'] != len(geometry['material_names'])):
        raise RuntimeError('reopened mesh material count does not match the build/manifest')
    for field, tolerance in (('actor_location_cm', 0.1), ('actor_rotation_rpy_deg', 1e-4), ('actor_scale', 1e-6)):
        if not _close_vector(observation[field], build[field], tolerance):
            raise RuntimeError(f'reopened actor transform changed: {field}')
