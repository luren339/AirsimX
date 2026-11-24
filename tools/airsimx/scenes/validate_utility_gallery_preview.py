"""Reopen and validate the isolated UE5 utility-gallery preview map."""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _preview_workflow import (DEFAULT_MAP, DEFAULT_OUTPUT, check_project_dir,
                               hash_existing_maps, map_file, write_report,
                               collision_counts, static_mesh_editor)

unreal = None


def validation_plan(args) -> dict:
    if not args.project_dir.is_dir():
        raise FileNotFoundError(f"UE project directory missing: {args.project_dir}")
    build = json.loads(args.build_report.read_text(encoding="utf-8"))
    if build.get("status") != "utility_gallery_preview_level_saved":
        raise ValueError("validation requires an actual UE build report, not a dry-run plan")
    if build["map_asset"] != args.map_asset:
        raise ValueError("build report points to a different map")
    target = map_file(args.project_dir, args.map_asset)
    if not target.is_file():
        raise FileNotFoundError(f"saved preview map missing: {target}")
    current = hash_existing_maps(args.project_dir, args.map_asset)
    if current != build["existing_map_sha256_before"]:
        raise RuntimeError("a protected pre-existing map changed since the build")
    return {"status": "utility_gallery_validation_planned", "engine_executed": False,
            "map_asset": args.map_asset, "map_file": str(target),
            "existing_map_sha256_after_reopen": current,
            "evidence_boundary": "input/hash checks only; the map has not been reopened in Unreal"}


def run_validation(args) -> dict:
    global unreal
    unreal = importlib.import_module("unreal")
    check_project_dir(args.project_dir, unreal)
    if not args.build_report.is_file():
        raise FileNotFoundError(f"builder report missing: {args.build_report}")
    build = json.loads(args.build_report.read_text(encoding="utf-8"))
    if build["map_asset"] != args.map_asset:
        raise RuntimeError(f"build report points to the wrong map: {build['map_asset']}")
    if not unreal.EditorLevelLibrary.load_level(args.map_asset):
        raise RuntimeError(f"could not reopen preview map {args.map_asset}")
    actors = list(unreal.EditorLevelLibrary.get_all_level_actors())
    structures = [actor for actor in actors if actor.get_actor_label() == "UtilityGallery_MainStructure"]
    lights = [actor for actor in actors if actor.get_actor_label().startswith("UtilityGallery_Light_")]
    player_starts = [actor for actor in actors if isinstance(actor, unreal.PlayerStart)]
    if len(structures) != 1:
        raise RuntimeError(f"expected one gallery structure actor after reopen, found {len(structures)}")
    if len(lights) != build["point_light_count"]:
        raise RuntimeError(f"point-light count changed after reopen: {len(lights)} != {build['point_light_count']}")
    if len(player_starts) != 1:
        raise RuntimeError(f"expected one preview PlayerStart, found {len(player_starts)}")

    structure = structures[0]
    component = structure.get_component_by_class(unreal.StaticMeshComponent)
    mesh = component.get_editor_property("static_mesh")
    if mesh is None:
        raise RuntimeError("gallery structure actor lost its StaticMesh after reopen")
    origin, extent = structure.get_actor_bounds(False)
    extent_cm = [float(extent.x), float(extent.y), float(extent.z)]
    if extent_cm[0] < 4000.0 or extent_cm[1] < 2500.0 or not 150.0 <= extent_cm[2] <= 300.0:
        raise RuntimeError(f"gallery bounds do not match a horizontal 3D utility corridor: {extent_cm}")
    body_setup = mesh.get_editor_property("body_setup")
    trace_flag = body_setup.get_editor_property("collision_trace_flag")
    trace_flag_text = str(trace_flag)
    # UE5.8 exposes this as an actor-component method, not an editor property.
    collision_enabled = str(component.get_collision_enabled())
    material_count = len(mesh.get_editor_property("static_materials"))
    if "CTF_USE_COMPLEX_AS_SIMPLE" not in trace_flag_text:
        raise RuntimeError(f"gallery mesh has unexpected collision mode: {trace_flag_text}")
    if "QUERY_AND_PHYSICS" not in collision_enabled:
        raise RuntimeError(f"gallery actor collision is not enabled for scene queries: {collision_enabled}")
    if material_count < 8:
        raise RuntimeError(f"gallery mesh lost imported material groups: {material_count}")
    simple, convex = collision_counts(static_mesh_editor(unreal), mesh)
    if simple != 0 or convex != 0:
        raise RuntimeError(f"simple/convex hulls reappeared after reopen: {simple}/{convex}")

    current_hashes = hash_existing_maps(args.project_dir, args.map_asset)
    existing_maps_unchanged = current_hashes == build["existing_map_sha256_before"]
    if not existing_maps_unchanged:
        raise RuntimeError("one or more pre-existing map files changed")
    report = {
        "status": "utility_gallery_preview_validated_after_reopen",
        "map_asset": args.map_asset,
        "map_reopened": True,
        "engine_executed": True,
        "engine_version": unreal.SystemLibrary.get_engine_version(),
        "actor_count": len(actors),
        "gallery_structure_actor_count": len(structures),
        "point_light_count": len(lights),
        "player_start_count": len(player_starts),
        "actor_bounds_origin_cm": [float(origin.x), float(origin.y), float(origin.z)],
        "actor_bounds_extent_cm": extent_cm,
        "static_mesh_asset": mesh.get_path_name(),
        "static_mesh_material_count": material_count,
        "collision_trace_flag": trace_flag_text,
        "component_collision_enabled": collision_enabled,
        "remaining_simple_collision_count": simple,
        "remaining_convex_collision_count": convex,
        "existing_maps_unchanged": existing_maps_unchanged,
        "existing_map_sha256_after_reopen": current_hashes,
        "four_loop_topology": build["loop_summary"],
        "clearance": build["clearance"],
        "evidence_boundary": "saved static UE5 preview reopened and checked; not an AirSim flight, sensor, SLAM, or collision-query experiment",
    }
    return report


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project-dir", type=Path, required=True)
    parser.add_argument("--map-asset", default=DEFAULT_MAP)
    parser.add_argument("--build-report", type=Path, default=DEFAULT_OUTPUT / "utility_gallery_preview_build.json")
    parser.add_argument("--report", type=Path, default=DEFAULT_OUTPUT / "utility_gallery_preview_validation.json")
    parser.add_argument("--dry-run", action="store_true", help="check inputs and map hashes without importing unreal")
    args = parser.parse_args(argv)
    plan = validation_plan(args)
    report = plan if args.dry_run else run_validation(args)
    write_report(args.report, report)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
