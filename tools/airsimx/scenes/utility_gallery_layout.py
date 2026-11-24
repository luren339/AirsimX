"""Deterministic layout model for the UE5 underground utility-gallery preview."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


DEFAULT_CONFIG = Path(__file__).resolve().parent / "settings" / "utility_gallery.json"
OPPOSITE = {"north": "south", "south": "north", "east": "west", "west": "east"}
VECTORS = {
    "north": (0.0, 1.0),
    "south": (0.0, -1.0),
    "east": (1.0, 0.0),
    "west": (-1.0, 0.0),
}


def _load_config(config: dict[str, Any] | str | Path | None) -> dict[str, Any]:
    if config is None:
        path = DEFAULT_CONFIG
        loaded = json.loads(path.read_text(encoding="utf-8"))
    elif isinstance(config, (str, Path)):
        loaded = json.loads(Path(config).read_text(encoding="utf-8"))
    else:
        loaded = config
    return loaded


def _edge_record(
    edge_id: str,
    start_id: str,
    end_id: str,
    start: tuple[float, float],
    end: tuple[float, float],
    direction: str,
    width: float,
    height: float,
    fixtures: dict[str, float],
    ordinal: int,
) -> dict[str, Any]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = abs(dx) + abs(dy)
    return {
        "id": edge_id,
        "from": start_id,
        "to": end_id,
        "direction": direction,
        "start_m": [round(start[0], 3), round(start[1], 3)],
        "end_m": [round(end[0], 3), round(end[1], 3)],
        "length_m": round(distance, 3),
        "width_m": width,
        "height_m": height,
        "structural_joint_spacing_m": round(
            fixtures["structural_joint_min"]
            + (ordinal * 0.71) % (fixtures["structural_joint_max"] - fixtures["structural_joint_min"]),
            3,
        ),
        "cable_support_spacing_m": round(
            fixtures["cable_support_min"]
            + (ordinal * 0.37) % (fixtures["cable_support_max"] - fixtures["cable_support_min"]),
            3,
        ),
        "light_spacing_m": round(
            fixtures["light_min"]
            + (ordinal * 0.83) % (fixtures["light_max"] - fixtures["light_min"]),
            3,
        ),
    }


def build_layout(config: dict[str, Any] | str | Path | None = None) -> dict[str, Any]:
    """Return validated nodes, corridors, and service spurs in local meters."""
    cfg = _load_config(config)
    columns = [float(value) for value in cfg["columns_x_m"]]
    rows = [float(value) for value in cfg["rows_y_m"]]
    dims = {key: float(value) for key, value in cfg["dimensions_m"].items()}
    envelope = {key: float(value) for key, value in cfg["flight_envelope_m"].items()}
    fixtures = {key: float(value) for key, value in cfg["fixture_spacing_m"].items()}

    if not all(math.isfinite(value) for value in columns + rows + list(dims.values())
               + list(envelope.values()) + list(fixtures.values())):
        raise ValueError("layout coordinates, dimensions and spacing must be finite")
    if any(value <= 0 for value in (*dims.values(), *envelope.values(), *fixtures.values())):
        raise ValueError("dimensions, envelope and fixture spacing must be positive")
    for name in ("structural_joint", "cable_support", "light"):
        if fixtures[f"{name}_max"] <= fixtures[f"{name}_min"]:
            raise ValueError(f"{name} spacing maximum must exceed minimum")
        if round(fixtures[f"{name}_min"], 3) <= 0:
            raise ValueError(f"{name} spacing must remain positive at millimeter precision")
    if len(columns) != 3 or len(rows) != 3:
        raise ValueError("utility gallery preview requires exactly three columns and rows")
    if any(b <= a for a, b in zip(columns, columns[1:])):
        raise ValueError("columns_x_m must be strictly increasing")
    if any(b <= a for a, b in zip(rows, rows[1:])):
        raise ValueError("rows_y_m must be strictly increasing")
    if dims["corridor_width"] < envelope["width"]:
        raise ValueError("flight envelope is wider than the corridor")
    if dims["interior_height"] < envelope["height"]:
        raise ValueError("flight envelope is taller than the corridor")
    if dims["junction_width"] <= max(dims["corridor_width"], dims["branch_width"]):
        raise ValueError("junction rooms must be strictly wider than corridors and branches")
    if dims["branch_width"] < envelope["width"]:
        raise ValueError("flight envelope is wider than a branch")
    if envelope["center_height"] - envelope["height"] / 2 < 0 or (
        envelope["center_height"] + envelope["height"] / 2
        > min(dims["interior_height"], dims["branch_height"], dims["junction_height"])
    ):
        raise ValueError("flight envelope must fit above the floor and below all ceilings")

    room_half = dims["junction_width"] / 2.0
    nodes = []
    node_lookup: dict[str, tuple[float, float]] = {}
    for row_index, y in enumerate(rows):
        for col_index, x in enumerate(columns):
            node_id = f"J{row_index}{col_index}"
            node_lookup[node_id] = (x, y)
            nodes.append(
                {
                    "id": node_id,
                    "position_m": [x, y],
                    "width_m": dims["junction_width"],
                    "height_m": dims["junction_height"],
                }
            )

    edges: list[dict[str, Any]] = []
    ordinal = 0
    for row_index in range(3):
        for col_index in range(2):
            start_id, end_id = f"J{row_index}{col_index}", f"J{row_index}{col_index + 1}"
            start_center, end_center = node_lookup[start_id], node_lookup[end_id]
            start = (start_center[0] + room_half, start_center[1])
            end = (end_center[0] - room_half, end_center[1])
            distance = end[0] - start[0]
            if distance <= 0.0:
                raise ValueError(f"junctions overlap along edge {start_id}-{end_id}")
            edges.append(
                _edge_record(
                    f"H{row_index}{col_index}", start_id, end_id, start, end,
                    "east", dims["corridor_width"], dims["interior_height"], fixtures, ordinal,
                )
            )
            ordinal += 1

    for col_index in range(3):
        for row_index in range(2):
            start_id, end_id = f"J{row_index}{col_index}", f"J{row_index + 1}{col_index}"
            start_center, end_center = node_lookup[start_id], node_lookup[end_id]
            start = (start_center[0], start_center[1] + room_half)
            end = (end_center[0], end_center[1] - room_half)
            distance = end[1] - start[1]
            if distance <= 0.0:
                raise ValueError(f"junctions overlap along edge {start_id}-{end_id}")
            edges.append(
                _edge_record(
                    f"V{row_index}{col_index}", start_id, end_id, start, end,
                    "north", dims["corridor_width"], dims["interior_height"], fixtures, ordinal,
                )
            )
            ordinal += 1

    occupied_exits = {node["id"]: set() for node in nodes}
    for edge in edges:
        occupied_exits[edge["from"]].add(edge["direction"])
        occupied_exits[edge["to"]].add(OPPOSITE[edge["direction"]])
    used_ids = {node["id"] for node in nodes} | {edge["id"] for edge in edges}
    branches = []
    for branch_cfg in cfg["branches"]:
        branch_id = branch_cfg["id"]
        if not isinstance(branch_id, str) or not branch_id or branch_id in used_ids:
            raise ValueError(f"branch requires a unique nonempty id: {branch_id}")
        node_id = branch_cfg["from_node"]
        if node_id not in node_lookup:
            raise ValueError(f"branch {branch_cfg['id']} references unknown node {node_id}")
        direction = branch_cfg["direction"]
        if direction not in VECTORS:
            raise ValueError(f"branch {branch_cfg['id']} has unknown direction {direction}")
        if direction in occupied_exits[node_id]:
            raise ValueError(f"branch exit already occupied: {node_id}/{direction}")
        vx, vy = VECTORS[direction]
        center = node_lookup[node_id]
        start = (center[0] + vx * room_half, center[1] + vy * room_half)
        length = float(branch_cfg["length_m"])
        if not math.isfinite(length) or length <= 0:
            raise ValueError("branch lengths must be finite and positive")
        used_ids.add(branch_id)
        occupied_exits[node_id].add(direction)
        end = (start[0] + vx * length, start[1] + vy * length)
        branches.append(
            {
                "id": branch_cfg["id"],
                "from_node": node_id,
                "direction": direction,
                "start_m": [round(start[0], 3), round(start[1], 3)],
                "end_m": [round(end[0], 3), round(end[1], 3)],
                "length_m": length,
                "width_m": dims["branch_width"],
                "height_m": dims["branch_height"],
            }
        )

    return {
        "schema_version": int(cfg["schema_version"]),
        "coordinate_units": "meters",
        "nodes": nodes,
        "edges": edges,
        "branches": branches,
        "dimensions_m": dims,
        "flight_envelope_m": envelope,
        "fixture_spacing_m": fixtures,
    }
