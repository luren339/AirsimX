"""Import the three generated visual meshes into explicitly selected UE folders."""

import argparse
import importlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _model_workflow import DEFAULT_ASSETS, DEFAULT_OUTPUT, import_plan, write_report


def run_import(plan):
    unreal = importlib.import_module('unreal')
    library = unreal.EditorAssetLibrary
    if not plan['replace_existing']:
        existing = [item['asset'] for item in plan['items'] if library.does_asset_exist(item['asset'])]
        if existing:
            raise RuntimeError(f'output meshes already exist; use a new asset root or --replace-existing: {existing}')
    records = []
    for item in plan['items']:
        task = unreal.AssetImportTask()
        for key, value in (
            ('filename', item['source']), ('destination_path', item['destination']),
            ('destination_name', item['asset'].rsplit('/', 1)[1]), ('automated', True),
            ('save', True), ('replace_existing', plan['replace_existing']),
            ('replace_existing_settings', plan['replace_existing']),
        ):
            task.set_editor_property(key, value)
        unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([task])
        paths = list(task.get_editor_property('imported_object_paths'))
        mesh = unreal.load_asset(item['asset'])
        if mesh is None or not isinstance(mesh, unreal.StaticMesh):
            raise RuntimeError(f'import returned no expected StaticMesh at {item["asset"]}: {paths}')
        bounds = mesh.get_bounds()
        records.append({**item, 'imported_object_paths': paths,
                        'bounds_cm': {'origin': [float(bounds.origin.x), float(bounds.origin.y), float(bounds.origin.z)],
                                      'extent': [float(bounds.box_extent.x), float(bounds.box_extent.y), float(bounds.box_extent.z)]}})
    for item in plan['items']:
        if not library.save_directory(item['destination'], only_if_is_dirty=False, recursive=True):
            raise RuntimeError(f'could not save imported mesh directory {item["destination"]}')
    return {**plan, 'status': 'sensor_models_imported', 'engine_executed': True,
            'engine_version': unreal.SystemLibrary.get_engine_version(), 'items': records,
            'evidence_boundary': 'static mesh import/bounds only; no Pawn, sensor or flight validation'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--generated-dir', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--asset-root', default=DEFAULT_ASSETS)
    parser.add_argument('--report', type=Path, default=DEFAULT_OUTPUT / 'generated_mesh_import.json')
    parser.add_argument('--replace-existing', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)
    plan = import_plan(args.generated_dir, args.asset_root, args.replace_existing)
    report = plan if args.dry_run else run_import(plan)
    write_report(args.report, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
