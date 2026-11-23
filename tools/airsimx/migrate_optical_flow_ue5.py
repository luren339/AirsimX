"""Run with UE5 PythonScriptPlugin; migrate only the legacy optical-flow projection."""

from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil

import unreal

ROOT = Path(__file__).resolve().parents[2]
OLD = 'float4 previousPos = mul(worldPos, View.PrevViewProj);'
NEW = ('float4 previousPos = mul(float4(worldPos.xyz + '
       'DFHackToFloat(ResolvedView.PrevPreViewTranslation), 1.0), '
       'ResolvedView.PrevTranslatedWorldToClip);')
MATERIALS = ('OpticalFlowMaterial', 'OpticalFlowRGBMaterial')
output = ROOT / 'build/airsimx/asset-migration' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
output.mkdir(parents=True, exist_ok=False)
report = {'status': 'running', 'materials': [], 'scope': 'Shader API migration only; not optical-flow accuracy validation'}


def save():
    (output / 'result.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf-8')


try:
    for name in MATERIALS:
        asset_path = '/AirSim/HUDAssets/' + name
        disk_path = ROOT / 'Unreal/Environments/Blocks/Plugins/AirSim/Content/HUDAssets' / (name + '.uasset')
        original_hash = hashlib.sha256(disk_path.read_bytes()).hexdigest()
        backup = output / (name + '.uasset')
        shutil.copy2(disk_path, backup)
        material = unreal.load_asset(asset_path)
        if material is None:
            raise RuntimeError('Material not found: ' + asset_path)
        expressions = unreal.MaterialEditingLibrary.get_material_expressions(material)
        candidates = [(expression, expression.get_editor_property('code')) for expression in expressions
                      if isinstance(expression, unreal.MaterialExpressionCustom)]
        old_matches = [(expression, code) for expression, code in candidates if OLD in code]
        new_matches = [(expression, code) for expression, code in candidates if NEW in code]
        if len(old_matches) == 1 and not new_matches:
            expression, code = old_matches[0]
            if code.count(OLD) != 1:
                raise RuntimeError('Ambiguous legacy shader expression: ' + name)
            expression.set_editor_property('code', code.replace(OLD, NEW))
            errors = unreal.MaterialEditingLibrary.recompile_material(material)
            if errors:
                expression.set_editor_property('code', code)
                raise RuntimeError('Shader compilation failed: ' + str(errors))
            if not unreal.EditorAssetLibrary.save_loaded_asset(material, only_if_is_dirty=False):
                raise RuntimeError('Asset save failed: ' + name)
            status = 'migrated'
        elif not old_matches and len(new_matches) == 1:
            status = 'already_migrated'
        else:
            raise RuntimeError('Unexpected custom expression layout: ' + name)
        report['materials'].append({'asset': asset_path, 'status': status, 'before_sha256': original_hash,
                                    'after_sha256': hashlib.sha256(disk_path.read_bytes()).hexdigest(),
                                    'backup': str(backup)})
        save()
    report['status'] = 'passed'
except Exception as error:
    report['status'] = 'failed'
    report['error'] = repr(error)
    raise
finally:
    save()
    unreal.log('AirsimX material migration result: ' + str(output / 'result.json'))
