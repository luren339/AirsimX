"""Path and input checks shared by the UE preview scripts (standard library only)."""

import hashlib
import json
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


def write_report(path, report):
    path = Path(path)
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
