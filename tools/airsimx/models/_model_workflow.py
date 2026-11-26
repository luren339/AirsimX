"""Portable, engine-free plans for the migrated model import/assembly scripts."""

import hashlib
import json
import math
from pathlib import Path
import re

DEFAULT_OUTPUT = Path(__file__).resolve().parents[3] / 'build/airsimx/models'
DEFAULT_ASSETS = '/Game/AirsimX/Drone'
DEFAULT_RIG = '/Game/AirsimX/Drone/BP_QuadrotorSensorRig'
DEFAULT_PARENT = '/AirSim/Blueprints/BP_FlyingPawn'
DEFAULT_CONFIG = Path(__file__).resolve().parent / 'settings/sensor_rig.json'
MODEL_FILES = (
    ('camera/approximate_camera_shell.obj', 'Camera'),
    ('lidar_mount/mid360s_mount_plate.obj', 'Mounts'),
    ('frame/OpenQuadrotorFrame30cm.obj', 'Models'),
)


def asset_path(path, game_only=False):
    if not isinstance(path, str) or not re.fullmatch(r'/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)+', path):
        raise ValueError(f'invalid Unreal package path: {path}')
    if game_only and not path.startswith('/Game/'):
        raise ValueError('generated assets must be under /Game/...')
    return path


def write_report(path, report):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def import_plan(generated_dir, asset_root=DEFAULT_ASSETS, replace_existing=False):
    asset_path(asset_root, game_only=True)
    items = []
    for relative, folder in MODEL_FILES:
        path = (Path(generated_dir) / relative).resolve()
        if not path.is_file():
            raise FileNotFoundError(f'generate the procedural model first: {path}')
        destination = asset_root + '/' + folder
        items.append({'source': str(path), 'destination': destination,
                      'asset': destination + '/' + path.stem,
                      'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    return {'status': 'sensor_model_import_planned', 'engine_executed': False,
            'asset_root': asset_root, 'replace_existing': replace_existing, 'items': items,
            'evidence_boundary': 'three procedural visual models; no third-party LiDAR CAD or engine validation'}


def _vector(values, label):
    if len(values) != 3 or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        raise ValueError(f'{label} must have three finite numbers')
    return values


def select_scene_root(records, parent_root_name=None):
    # Non-scene components (e.g. RotatingMovementComponent) can also carry the
    # editor root flag. An uninstantiated Blueprint CDO may have no scene root.
    scene = [record for record in records if record['is_scene_component']]
    if parent_root_name is not None:
        matches = [record for record in scene
                   if parent_root_name in (record['variable_name'], record['object_name'])]
    else:
        matches = [record for record in scene if record['is_root_component']]
    if len(matches) != 1:
        raise RuntimeError(f'expected one scene root (parent CDO name {parent_root_name}), found {len(matches)}')
    return matches[0]


def rig_plan(config=DEFAULT_CONFIG, asset_root=DEFAULT_ASSETS, blueprint_asset=DEFAULT_RIG,
             parent_blueprint=DEFAULT_PARENT, replace_existing=False):
    asset_path(asset_root, game_only=True)
    asset_path(blueprint_asset, game_only=True)
    asset_path(parent_blueprint)
    if blueprint_asset == parent_blueprint:
        raise ValueError('the derived blueprint must differ from its parent')
    data = json.loads(Path(config).read_text(encoding='utf-8'))
    if data.get('schema_version') != 1 or data.get('units') != 'centimeters':
        raise ValueError('expected schema_version=1 and centimeter transforms')
    components = []
    names = set()
    for entry in data['components']:
        name = entry['name']
        if not re.fullmatch(r'AirsimX[A-Za-z0-9_]+', name) or name in names:
            raise ValueError('visual component names must be unique and start with AirsimX')
        names.add(name)
        if 'asset' in entry:
            mesh = asset_path(entry['asset'])  # Explicit external UE resource, never bundled implicitly.
        else:
            relative = entry['asset_relative_path']
            mesh = asset_path(asset_root + '/' + relative, game_only=True)
        scale = _vector(entry['scale'], 'scale')
        if any(value <= 0 for value in scale):
            raise ValueError('visual component scales must be positive')
        components.append({'name': name, 'asset': mesh,
                           'location_cm': _vector(entry['location_cm'], 'location_cm'),
                           'rotation_rpy_deg': _vector(entry['rotation_rpy_deg'], 'rotation_rpy_deg'),
                           'scale': scale, 'collision': 'NoCollision'})
    if not components:
        raise ValueError('a rig needs at least one visual component')
    return {'status': 'sensor_rig_blueprint_planned', 'engine_executed': False,
            'blueprint_asset': blueprint_asset, 'parent_blueprint': parent_blueprint,
            'replace_existing': replace_existing, 'components': components,
            'evidence_boundary': 'visual attachments only; inherited sensors, dynamics, scale and collision are not calibrated'}
