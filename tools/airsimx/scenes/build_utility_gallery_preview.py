"""Import the generated gallery GLB into an isolated UE5 preview level."""

from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
from pathlib import Path

# Unreal's ExecutePythonScript does not consistently add the script directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from _preview_workflow import (DEFAULT_ASSETS, DEFAULT_MAP, DEFAULT_OUTPUT,
                               check_project_dir, game_path, hash_existing_maps,
                               map_file, metadata_to_ue_cm, read_geometry, write_report,
                               collision_counts, static_mesh_editor)

unreal = None  # Loaded only after explicit CLI input checks, never on import.
LIGHT_INTENSITY = 450.0
LIGHT_RADIUS_CM = 1200.0
LIGHT_SOURCE_RADIUS_CM = 135.0


def import_gallery_mesh(args):
    if not args.glb.is_file() or not args.metadata.is_file():
        raise FileNotFoundError("generate the gallery GLB and geometry manifest before running the UE builder")
    import_task = unreal.AssetImportTask()
    import_task.set_editor_property("filename", str(args.glb))
    import_task.set_editor_property("destination_path", args.asset_root)
    import_task.set_editor_property("automated", True)
    import_task.set_editor_property("replace_existing", args.replace_existing)
    import_task.set_editor_property("replace_existing_settings", args.replace_existing)
    import_task.set_editor_property("save", True)
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([import_task])
    imported_paths = list(import_task.get_editor_property("imported_object_paths"))

    static_meshes = []
    for path in imported_paths:
        asset = unreal.EditorAssetLibrary.load_asset(path)
        if isinstance(asset, unreal.StaticMesh):
            static_meshes.append((path, asset))
    if not static_meshes:
        registry = unreal.AssetRegistryHelpers.get_asset_registry()
        for asset_data in registry.get_assets_by_path(args.asset_root, recursive=True):
            asset = asset_data.get_asset()
            if isinstance(asset, unreal.StaticMesh):
                static_meshes.append((asset.get_path_name(), asset))
    if not static_meshes:
        raise RuntimeError(f"GLB import returned no StaticMesh assets: {imported_paths}")
    preferred = [item for item in static_meshes if "UtilityGalleryPreview" in item[0] or "utility_gallery_preview" in item[0].lower()]
    if len(preferred) == 1:
        mesh_path, mesh = preferred[0]
    elif len(static_meshes) == 1:
        mesh_path, mesh = static_meshes[0]
    else:
        raise RuntimeError(f"expected one gallery StaticMesh, found: {[path for path, _ in static_meshes]}")
    return mesh_path, mesh, imported_paths


def configure_mesh_collision(mesh_path: str, mesh) -> dict:
    # This gallery is one hollow, multi-room mesh. Interchange may import an
    # auto-generated convex hull that fills its walkable interior, so keep the
    # real triangle mesh as the only collision representation.
    editor = static_mesh_editor(unreal)
    if not editor.remove_collisions(mesh):
        raise RuntimeError(f"could not remove imported collisions: {mesh_path}")
    simple, convex = collision_counts(editor, mesh)
    if simple != 0 or convex != 0:
        raise RuntimeError(f"imported collision remains on hollow gallery mesh: simple={simple}, convex={convex}: {mesh_path}")
    body_setup = mesh.get_editor_property("body_setup")
    if body_setup is None:
        raise RuntimeError(f"StaticMesh has no BodySetup: {mesh_path}")
    body_setup.set_editor_property("collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_COMPLEX_AS_SIMPLE)
    if not unreal.EditorAssetLibrary.save_asset(mesh_path):
        raise RuntimeError(f"could not save collision settings for {mesh_path}")
    return {
        "trace_flag": "UseComplexAsSimple",
        "imported_simple_collision_removed": True,
        "remaining_simple_collision_count": 0,
        "remaining_convex_collision_count": 0,
        "enabled_on_component": True,
    }


def load_or_create_level(args) -> None:
    if unreal.EditorAssetLibrary.does_asset_exist(args.map_asset):
        if not unreal.EditorLevelLibrary.load_level(args.map_asset):
            raise RuntimeError(f"could not load existing preview map {args.map_asset}")
        for actor in unreal.EditorLevelLibrary.get_all_level_actors():
            if actor.get_actor_label().startswith("UtilityGallery_"):
                unreal.EditorLevelLibrary.destroy_actor(actor)
    elif not unreal.EditorLevelLibrary.new_level(args.map_asset):
        raise RuntimeError(f"could not create preview map {args.map_asset}")


def spawn_gallery_actor(mesh):
    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.StaticMeshActor, unreal.Vector(0.0, 0.0, 0.0), unreal.Rotator(0.0, 0.0, 0.0)
    )
    if actor is None:
        raise RuntimeError("failed to spawn UtilityGallery mesh actor")
    actor.set_actor_label("UtilityGallery_MainStructure")
    component = actor.get_component_by_class(unreal.StaticMeshComponent)
    component.set_static_mesh(mesh)
    component.set_collision_profile_name("BlockAll")
    component.set_collision_enabled(unreal.CollisionEnabled.QUERY_AND_PHYSICS)
    return actor


def spawn_lights(metadata: dict, args) -> int:
    count = 0
    for item in metadata["lights"]:
        x, source_y, z = (float(value) * 100.0 for value in item["position_m"])
        # GLB import maps gallery +Y to UE -Y; place lights in that same frame.
        y = -source_y
        actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
            unreal.PointLight, unreal.Vector(x, y, z), unreal.Rotator(0.0, 0.0, 0.0)
        )
        if actor is None:
            raise RuntimeError(f"failed to spawn light {item['id']}")
        actor.set_actor_label(f"UtilityGallery_Light_{item['id']}")
        component = actor.get_component_by_class(unreal.PointLightComponent)
        component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
        component.set_editor_property("intensity", args.light_intensity)
        component.set_editor_property("attenuation_radius", LIGHT_RADIUS_CM)
        component.set_editor_property("source_radius", LIGHT_SOURCE_RADIUS_CM)
        component.set_editor_property("use_inverse_squared_falloff", True)
        # A low-light enclosed level with no baked GI otherwise clips all pipe
        # shadows to black. Let textured surfaces carry the detail while the
        # practical fixtures provide an evenly readable preview.
        component.set_editor_property("cast_shadows", False)
        component.set_editor_property("light_color", unreal.Color(238, 232, 218, 255))
        count += 1
    return count


def spawn_preview_exposure() -> dict:
    volume = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.PostProcessVolume, unreal.Vector(0.0, 0.0, 0.0), unreal.Rotator(0.0, 0.0, 0.0)
    )
    if volume is None:
        raise RuntimeError("failed to create the preview PostProcessVolume")
    volume.set_actor_label("UtilityGallery_PostProcess_ManualExposure")
    volume.set_editor_property("unbound", True)
    settings = volume.get_editor_property("settings")
    settings.set_editor_property("override_auto_exposure_method", True)
    settings.set_editor_property("auto_exposure_method", unreal.AutoExposureMethod.AEM_MANUAL)
    settings.set_editor_property("override_auto_exposure_bias", True)
    settings.set_editor_property("auto_exposure_bias", 3.0)
    volume.set_editor_property("settings", settings)
    return {
        "unbound": True,
        "auto_exposure_method": "Manual",
        "auto_exposure_bias_ev": 3.0,
        "scope": "preview level only",
    }


def spawn_player_start(position_m):
    actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.PlayerStart,
        unreal.Vector(*metadata_to_ue_cm(position_m)),
        unreal.Rotator(0.0, 0.0, 0.0),
    )
    if actor is None:
        raise RuntimeError("failed to create gallery PlayerStart")
    actor.set_actor_label("UtilityGallery_PlayerStart")


def build_plan(args) -> dict:
    if not args.project_dir.is_dir():
        raise FileNotFoundError(f"UE project directory missing: {args.project_dir}")
    game_path(args.asset_root)
    if not math.isfinite(args.light_intensity) or args.light_intensity <= 0:
        raise ValueError("light intensity must be finite and positive")
    geometry = read_geometry(args.glb, args.metadata)
    return {
        "status": "utility_gallery_preview_planned", "engine_executed": False,
        "map_asset": game_path(args.map_asset),
        "map_file": str(map_file(args.project_dir, args.map_asset)),
        "asset_root": args.asset_root, "glb_source": str(args.glb.resolve()),
        "glb_sha256": geometry["glb_sha256"],
        "geometry_metadata": str(args.metadata.resolve()),
        "existing_map_sha256_before": hash_existing_maps(args.project_dir, args.map_asset),
        "replace_existing": args.replace_existing,
        "evidence_boundary": "plan/input validation only; no Unreal assets imported or level saved",
    }


def run_build(args, plan) -> dict:
    global unreal
    unreal = importlib.import_module("unreal")
    check_project_dir(args.project_dir, unreal)
    if not args.replace_existing:
        if unreal.EditorAssetLibrary.does_asset_exist(args.map_asset):
            raise RuntimeError("target map already exists; choose a new path or pass --replace-existing")
        if unreal.EditorAssetLibrary.list_assets(args.asset_root, recursive=True, include_folder=False):
            raise RuntimeError("target asset root is nonempty; choose a new root or pass --replace-existing")
    geometry = read_geometry(args.glb, args.metadata)
    mesh_path, mesh, imported_paths = import_gallery_mesh(args)
    collision = configure_mesh_collision(mesh_path, mesh)
    load_or_create_level(args)
    structure = spawn_gallery_actor(mesh)
    light_count = spawn_lights(geometry, args)
    exposure = spawn_preview_exposure()
    spawn_player_start(geometry["player_start_m"])
    if not unreal.EditorLevelLibrary.save_current_level():
        raise RuntimeError(f"failed to save preview map {args.map_asset}")
    if not unreal.EditorAssetLibrary.does_asset_exist(args.map_asset):
        raise RuntimeError(f"saved preview map is not registered: {args.map_asset}")
    after = hash_existing_maps(args.project_dir, args.map_asset)
    if after != plan["existing_map_sha256_before"]:
        raise RuntimeError("a pre-existing map package changed during preview build")
    origin, extent = structure.get_actor_bounds(False)
    return {
        **plan, "status": "utility_gallery_preview_level_saved", "engine_executed": True,
        "engine_version": unreal.SystemLibrary.get_engine_version(),
        "project_dir": str(args.project_dir.resolve()),
        "imported_object_paths": imported_paths, "static_mesh_asset": mesh_path,
        "static_mesh_material_count": len(mesh.get_editor_property("static_materials")),
        "actor_label": structure.get_actor_label(),
        "actor_bounds_origin_cm": [float(origin.x), float(origin.y), float(origin.z)],
        "actor_bounds_extent_cm": [float(extent.x), float(extent.y), float(extent.z)],
        "collision": collision, "point_light_count": light_count,
        "light_coordinate_transform": "metadata meters (x,y,z) -> UE centimeters (100x,-100y,100z)",
        "lighting": {"point_light_lumens": args.light_intensity,
                     "attenuation_radius_cm": LIGHT_RADIUS_CM, "mobility": "Movable",
                     "cast_shadows": False, "source_radius_cm": LIGHT_SOURCE_RADIUS_CM,
                     "post_process": exposure},
        "player_start": metadata_to_ue_cm(geometry["player_start_m"]),
        "loop_summary": geometry["layout_summary"], "clearance": geometry["clearance"],
        "existing_map_sha256_after": after, "existing_maps_unchanged": True,
        "evidence_boundary": "saved static preview; no flight, sensor, SLAM or collision-query experiment",
    }


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-dir", type=Path, required=True, help="directory containing the active .uproject")
    parser.add_argument("--glb", type=Path, default=DEFAULT_OUTPUT / "utility_gallery_preview.glb")
    parser.add_argument("--metadata", type=Path, default=DEFAULT_OUTPUT / "utility_gallery_geometry.json")
    parser.add_argument("--map-asset", default=DEFAULT_MAP)
    parser.add_argument("--asset-root", default=DEFAULT_ASSETS)
    parser.add_argument("--report", type=Path, default=DEFAULT_OUTPUT / "utility_gallery_preview_build.json")
    parser.add_argument("--light-intensity", type=float, default=LIGHT_INTENSITY)
    parser.add_argument("--replace-existing", action="store_true", help="explicitly update the chosen generated map/assets")
    parser.add_argument("--dry-run", action="store_true", help="write an input/import plan without importing unreal")
    return parser.parse_args(argv)


def main(argv=None) -> None:
    args = parse_args(argv)
    plan = build_plan(args)
    report = plan if args.dry_run else run_build(args, plan)
    write_report(args.report, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
