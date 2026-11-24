"""Generate a procedural, multi-loop underground utility-gallery GLB."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import struct
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from utility_gallery_layout import VECTORS, build_layout


REPO_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_OUTPUT = REPO_ROOT / "build" / "airsimx" / "scenes"
DEFAULT_GLB = DEFAULT_OUTPUT / "utility_gallery_preview.glb"
DEFAULT_METADATA = DEFAULT_OUTPUT / "utility_gallery_geometry.json"
TEXTURE_FILES = ("shaft_albedo.png", "shaft_normal.png", "shaft_roughness.png")
MATERIALS = (
    {
        "name": "Concrete_WarmGrey",
        "baseColorFactor": [0.67, 0.66, 0.63, 1.0],
        "metallicFactor": 0.0,
        "roughnessFactor": 0.94,
        "texture": "concrete",
    },
    {"name": "Concrete_Trim", "baseColorFactor": [0.56, 0.55, 0.52, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.92, "texture": "concrete"},
    {"name": "Concrete_Joint", "baseColorFactor": [0.43, 0.42, 0.39, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.96},
    {"name": "Coated_Steel", "baseColorFactor": [0.30, 0.31, 0.29, 1.0], "metallicFactor": 0.18, "roughnessFactor": 0.88},
    {"name": "Cable_Jacket", "baseColorFactor": [0.30, 0.31, 0.30, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.86},
    {"name": "Pipe_Insulation", "baseColorFactor": [0.31, 0.30, 0.28, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.94},
    {"name": "Pipe_WaterBlue", "baseColorFactor": [0.12, 0.24, 0.36, 1.0], "metallicFactor": 0.08, "roughnessFactor": 0.88},
    {"name": "Pipe_FireRed", "baseColorFactor": [0.36, 0.11, 0.08, 1.0], "metallicFactor": 0.04, "roughnessFactor": 0.90},
    {"name": "Pipe_ServiceGreen", "baseColorFactor": [0.11, 0.25, 0.15, 1.0], "metallicFactor": 0.04, "roughnessFactor": 0.90},
    {"name": "Pipe_Coupling", "baseColorFactor": [0.50, 0.51, 0.48, 1.0], "metallicFactor": 0.14, "roughnessFactor": 0.78},
    {"name": "Service_Trim", "baseColorFactor": [0.62, 0.63, 0.60, 1.0], "metallicFactor": 0.04, "roughnessFactor": 0.84},
    {"name": "Safety_Marking", "baseColorFactor": [0.43, 0.29, 0.095, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.88},
    {"name": "Light_Cover", "baseColorFactor": [0.55, 0.53, 0.48, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.82},
    {"name": "Signage_Panel", "baseColorFactor": [0.70, 0.71, 0.68, 1.0], "metallicFactor": 0.0, "roughnessFactor": 0.96, "emissiveFactor": [0.14, 0.14, 0.13]},
)
MATERIAL_INDEX = {material["name"]: index for index, material in enumerate(MATERIALS)}


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _normalize(vector):
    length = math.sqrt(sum(value * value for value in vector))
    if length < 1e-10:
        raise ValueError("degenerate triangle in utility-gallery geometry")
    return tuple(value / length for value in vector)


def _png(image: np.ndarray) -> bytes:
    success, encoded = cv2.imencode(".png", image)
    if not success:
        raise RuntimeError("PNG encoding failed")
    return encoded.tobytes()


def _concrete_material_textures(texture_dir: str | Path | None = None) -> tuple[bytes, bytes, bytes]:
    """Use original atlas cropping only for explicit external inputs.

    The default is a deterministic code-generated matte material. No reference
    texture, vendor image or platform font is searched for implicitly.
    """
    if texture_dir is None:
        noise = np.random.default_rng(0).integers(-8, 9, (128, 128), dtype=np.int16)
        grey = np.clip(190 + noise, 0, 255).astype(np.uint8)
        albedo = np.repeat(grey[:, :, None], 3, axis=2)
        # OpenCV uses BGR(A); PNG encoding writes the RGB(A) channels correctly.
        normal = np.empty((128, 128, 3), dtype=np.uint8)
        normal[:] = (255, 128, 128)
        roughness = np.full((128, 128), 238, dtype=np.uint8)
    else:
        texture_dir = Path(texture_dir)
        sources = [texture_dir / name for name in TEXTURE_FILES]
        missing = [str(path) for path in sources if not path.is_file()]
        if missing:
            raise FileNotFoundError(f"external concrete PBR maps are missing: {missing}")
        maps = []
        # Retain the source's concrete-panel crop for compatible external atlases.
        for index, path in enumerate(sources):
            flags = cv2.IMREAD_GRAYSCALE if index == 2 else cv2.IMREAD_COLOR
            image = cv2.imdecode(np.frombuffer(path.read_bytes(), np.uint8), flags)
            if image is None or image.shape[0] < 662 or image.shape[1] < 852:
                raise ValueError(f"external texture must be a readable image at least 852x662: {path}")
            maps.append(image[150:662, 340:852].copy())
        albedo, normal, roughness = maps
        roughness = np.maximum(roughness, 225)
    packed = np.zeros((*roughness.shape, 4), dtype=np.uint8)
    packed[:, :, 1] = roughness  # glTF roughness in G, metallic in B.
    packed[:, :, 3] = 255
    return _png(albedo), _png(normal), _png(packed)


def _signage_atlas_png() -> bytes:
    # Hershey lettering ships with OpenCV; no OS font files are required.
    image = np.full((768, 768, 3), (103, 108, 104), dtype=np.uint8)
    for row in range(3):
        for column in range(3):
            left, top = column * 256, row * 256
            cv2.rectangle(image, (left + 8, top + 8), (left + 247, top + 247), (151, 160, 158), 2)
            cv2.rectangle(image, (left + 16, top + 18), (left + 21, top + 238), (66, 128, 166), -1)
            for text, y, scale, color in (
                ("UTILITY ACCESS", 49, 0.57, (195, 207, 208)),
                (f"J{row}{column}", 119, 1.65, (202, 216, 220)),
                ("SERVICE ZONE", 167, 0.45, (174, 184, 183)),
            ):
                cv2.putText(image, text, (left + 32, top + y), cv2.FONT_HERSHEY_SIMPLEX,
                            scale, color, 1, cv2.LINE_AA)
            cv2.arrowedLine(image, (left + 32, top + 188), (left + 235, top + 188),
                           (83, 142, 174), 3, tipLength=0.05)
    return _png(image)


class _SceneMesh:
    def __init__(self):
        self.groups: dict[str, dict[str, list]] = defaultdict(lambda: {"positions": [], "normals": [], "uvs": [], "indices": []})
        self.component_counts = Counter()
        self.triangle_count = 0
        self.bounds_min = [float("inf")] * 3
        self.bounds_max = [float("-inf")] * 3
        self.lights: list[dict] = []
        self.clearance_records: list[dict] = []
        self.clearance_violations: list[dict] = []
        self.lateral_margins: list[float] = []
        self.vertical_margins: list[float] = []
        self.service_routing = {
            "unconnected_pipe_ends": 0,
            "junction_fitting_count": 0,
            "junction_port_coupling_count": 0,
            "branch_lane_transition_count": 0,
            "branch_wall_sleeve_count": 0,
            "branch_wall_penetration_count": 0,
            "branch_wall_escutcheon_count": 0,
            "branch_terminal_cap_count": 0,
            "cable_junction_box_count": 0,
            "portal_return_pier_count": 0,
            "junction_service_return_count": 0,
            "junctions": [],
            "junction_count": 0,
        }

    @staticmethod
    def _uv(point, normal, scale=0.18):
        if abs(normal[2]) >= abs(normal[0]) and abs(normal[2]) >= abs(normal[1]):
            axes = (0, 1)
        elif abs(normal[1]) >= abs(normal[0]):
            axes = (0, 2)
        else:
            axes = (1, 2)
        return (point[axes[0]] * scale, point[axes[1]] * scale)

    def _triangle(self, material: str, p0, p1, p2, uv0=None, uv1=None, uv2=None, vertex_normals=None):
        ab = tuple(p1[i] - p0[i] for i in range(3))
        ac = tuple(p2[i] - p0[i] for i in range(3))
        normal = _normalize(_cross(ab, ac))
        group = self.groups[material]
        points = (p0, p1, p2)
        uvs = (uv0, uv1, uv2)
        base = len(group["positions"])
        for index, point in enumerate(points):
            point = tuple(float(value) for value in point)
            group["positions"].append(point)
            group["normals"].append(_normalize(vertex_normals[index]) if vertex_normals is not None else normal)
            group["uvs"].append(uvs[index] if uvs[index] is not None else self._uv(point, normal))
            for axis in range(3):
                self.bounds_min[axis] = min(self.bounds_min[axis], point[axis])
                self.bounds_max[axis] = max(self.bounds_max[axis], point[axis])
        group["indices"].extend((base, base + 1, base + 2))
        self.triangle_count += 1

    def quad(self, material: str, points: Iterable[tuple[float, float, float]], uv_points=None, normal_points=None):
        points = tuple(points)
        if len(points) != 4:
            raise ValueError("quad must contain four vertices")
        uv_points = tuple(uv_points) if uv_points is not None else (None,) * 4
        normals = tuple(normal_points) if normal_points is not None else (None,) * 4
        self._triangle(material, points[0], points[1], points[2], uv_points[0], uv_points[1], uv_points[2], normals[:3] if normal_points is not None else None)
        self._triangle(material, points[0], points[2], points[3], uv_points[0], uv_points[2], uv_points[3], (normals[0], normals[2], normals[3]) if normal_points is not None else None)

    def box(self, center, size, material: str, component: str, yaw: float = 0.0):
        cx, cy, cz = center
        sx, sy, sz = (value / 2.0 for value in size)
        c, s = math.cos(yaw), math.sin(yaw)
        local = [
            (-sx, -sy, -sz), (sx, -sy, -sz), (sx, sy, -sz), (-sx, sy, -sz),
            (-sx, -sy, sz), (sx, -sy, sz), (sx, sy, sz), (-sx, sy, sz),
        ]
        points = [(cx + x * c - y * s, cy + x * s + y * c, cz + z) for x, y, z in local]
        faces = (
            (0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
            (3, 7, 6, 2), (0, 4, 7, 3), (1, 2, 6, 5),
        )
        for face in faces:
            p = [points[index] for index in face]
            normal = _normalize(_cross(tuple(p[1][i] - p[0][i] for i in range(3)), tuple(p[2][i] - p[0][i] for i in range(3))))
            uvs = [self._uv(point, normal) for point in p]
            self.quad(material, p, uvs)
        self.component_counts[component] += 1

    def plane(self, material: str, points, uv_points, component: str):
        self.quad(material, points, uv_points)
        self.component_counts[component] += 1

    def cylinder(
        self,
        start,
        end,
        radius: float,
        material: str,
        component: str,
        sides: int = 48,
        cap_start: bool = True,
        cap_end: bool = True,
    ):
        dx, dy, dz = (end[i] - start[i] for i in range(3))
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-8:
            raise ValueError(f"zero-length cylinder for {component}")
        axis = (dx / length, dy / length, dz / length)
        helper = (0.0, 0.0, 1.0) if abs(axis[2]) < 0.9 else (1.0, 0.0, 0.0)
        basis_a = _normalize(_cross(axis, helper))
        basis_b = _normalize(_cross(axis, basis_a))
        ring_a, ring_b, radial_normals = [], [], []
        for index in range(sides):
            angle = 2.0 * math.pi * index / sides
            radial = tuple(basis_a[k] * math.cos(angle) + basis_b[k] * math.sin(angle) for k in range(3))
            radial_normals.append(radial)
            ring_a.append(tuple(start[k] + radial[k] * radius for k in range(3)))
            ring_b.append(tuple(end[k] + radial[k] * radius for k in range(3)))
        for index in range(sides):
            nxt = (index + 1) % sides
            self.quad(material, (ring_a[index], ring_a[nxt], ring_b[nxt], ring_b[index]),
                      normal_points=(radial_normals[index], radial_normals[nxt], radial_normals[nxt], radial_normals[index]))
            if cap_start:
                self._triangle(material, start, ring_a[nxt], ring_a[index])
            if cap_end:
                self._triangle(material, end, ring_b[index], ring_b[nxt])
        self.component_counts[component] += 1

    def tube_path(
        self,
        points,
        radius: float,
        material: str,
        component: str,
        sides: int = 48,
        cap_start: bool = True,
        cap_end: bool = True,
    ):
        """Create a capped tube along a short polyline without visible segment joints."""
        points = [tuple(float(value) for value in point) for point in points]
        if len(points) < 2:
            raise ValueError(f"tube path for {component} needs at least two points")
        directions = [_normalize(tuple(points[i + 1][k] - points[i][k] for k in range(3))) for i in range(len(points) - 1)]
        if any(math.dist(points[i], points[i + 1]) < 1e-6 for i in range(len(points) - 1)):
            raise ValueError(f"zero-length tube section for {component}")
        tangents = [directions[0]]
        for index in range(1, len(points) - 1):
            tangent = tuple(directions[index - 1][k] + directions[index][k] for k in range(3))
            tangents.append(_normalize(tangent))
        tangents.append(directions[-1])

        rings = []
        ring_normals = []
        for point, tangent in zip(points, tangents):
            helper = (0.0, 0.0, 1.0) if abs(tangent[2]) < 0.9 else (1.0, 0.0, 0.0)
            basis_a = _normalize(_cross(tangent, helper))
            basis_b = _normalize(_cross(tangent, basis_a))
            ring, normals = [], []
            for index in range(sides):
                angle = 2.0 * math.pi * index / sides
                radial = _normalize(tuple(basis_a[k] * math.cos(angle) + basis_b[k] * math.sin(angle) for k in range(3)))
                ring.append(tuple(point[k] + radial[k] * radius for k in range(3)))
                normals.append(radial)
            rings.append(ring)
            ring_normals.append(normals)
        for ring_index in range(len(rings) - 1):
            for side_index in range(sides):
                nxt = (side_index + 1) % sides
                self.quad(
                    material,
                    (rings[ring_index][side_index], rings[ring_index][nxt], rings[ring_index + 1][nxt], rings[ring_index + 1][side_index]),
                    normal_points=(ring_normals[ring_index][side_index], ring_normals[ring_index][nxt], ring_normals[ring_index + 1][nxt], ring_normals[ring_index + 1][side_index]),
                )
        start_center, end_center = points[0], points[-1]
        for side_index in range(sides):
            nxt = (side_index + 1) % sides
            if cap_start:
                self._triangle(material, start_center, rings[0][nxt], rings[0][side_index])
            if cap_end:
                self._triangle(material, end_center, rings[-1][side_index], rings[-1][nxt])
        self.component_counts[component] += 1

    def sphere(self, center, radius: float, material: str, component: str, sides: int = 24, rings: int = 8):
        """Create a smooth, small service coupling hub around a multi-port join."""
        cx, cy, cz = center
        latitudes = [-math.pi / 2.0 + math.pi * (index + 1) / (rings + 1) for index in range(rings)]
        points_by_ring, normals_by_ring = [], []
        for latitude in latitudes:
            points, normals = [], []
            for index in range(sides):
                longitude = 2.0 * math.pi * index / sides
                normal = (math.cos(latitude) * math.cos(longitude), math.cos(latitude) * math.sin(longitude), math.sin(latitude))
                normals.append(normal)
                points.append((cx + radius * normal[0], cy + radius * normal[1], cz + radius * normal[2]))
            points_by_ring.append(points)
            normals_by_ring.append(normals)
        for ring_index in range(rings - 1):
            for side_index in range(sides):
                nxt = (side_index + 1) % sides
                self.quad(
                    material,
                    (points_by_ring[ring_index][side_index], points_by_ring[ring_index][nxt], points_by_ring[ring_index + 1][nxt], points_by_ring[ring_index + 1][side_index]),
                    normal_points=(normals_by_ring[ring_index][side_index], normals_by_ring[ring_index][nxt], normals_by_ring[ring_index + 1][nxt], normals_by_ring[ring_index + 1][side_index]),
                )
        bottom = (cx, cy, cz - radius)
        top = (cx, cy, cz + radius)
        for side_index in range(sides):
            nxt = (side_index + 1) % sides
            self._triangle(material, bottom, points_by_ring[0][nxt], points_by_ring[0][side_index])
            self._triangle(material, top, points_by_ring[-1][side_index], points_by_ring[-1][nxt])
        self.component_counts[component] += 1

    def annular_ring_on_axis(self, center, axis_yaw, inner_radius, outer_radius,
                             material: str, component: str, sides: int = 48):
        """Create a flat annular escutcheon facing back along a corridor axis."""
        if inner_radius <= 0.0 or outer_radius <= inner_radius:
            raise ValueError("annular ring radii must satisfy 0 < inner < outer")
        cx, cy, cz = center
        lateral = (-math.sin(axis_yaw), math.cos(axis_yaw))
        for index in range(sides):
            angle_a = 2.0 * math.pi * index / sides
            angle_b = 2.0 * math.pi * (index + 1) / sides
            def point(radius, angle):
                return (cx + lateral[0] * radius * math.cos(angle),
                        cy + lateral[1] * radius * math.cos(angle),
                        cz + radius * math.sin(angle))
            outer_a, outer_b = point(outer_radius, angle_a), point(outer_radius, angle_b)
            inner_a, inner_b = point(inner_radius, angle_a), point(inner_radius, angle_b)
            normal = (-math.cos(axis_yaw), -math.sin(axis_yaw), 0.0)
            self.quad(material, (outer_a, inner_a, inner_b, outer_b),
                      normal_points=(normal, normal, normal, normal))
        self.component_counts[component] += 1

    def disk_on_axis(self, center, axis_yaw, radius, material: str,
                     component: str, sides: int = 48):
        """Create a flat service blind plate facing toward the corridor."""
        cx, cy, cz = center
        normal = (-math.cos(axis_yaw), -math.sin(axis_yaw), 0.0)
        ring = []
        for index in range(sides):
            angle = 2.0 * math.pi * index / sides
            ring.append((cx - math.sin(axis_yaw) * radius * math.cos(angle),
                         cy + math.cos(axis_yaw) * radius * math.cos(angle),
                         cz + radius * math.sin(angle)))
        for index in range(sides):
            nxt = (index + 1) % sides
            self._triangle(material, (cx, cy, cz), ring[nxt], ring[index],
                           vertex_normals=(normal, normal, normal))
        self.component_counts[component] += 1

    def domed_cap_on_axis(self, base_center, axis_yaw, radius, material: str,
                          component: str, sides: int = 48, rings: int = 10):
        """Close a pipe with a corridor-facing hemispherical cap."""
        cx, cy, cz = base_center
        axis = (math.cos(axis_yaw), math.sin(axis_yaw), 0.0)
        lateral = (-math.sin(axis_yaw), math.cos(axis_yaw), 0.0)
        apex = (cx - axis[0] * radius, cy - axis[1] * radius, cz)
        apex_normal = (-axis[0], -axis[1], 0.0)
        theta_values = [math.pi * 0.5 * index / rings for index in range(1, rings + 1)]
        ring_points, ring_normals = [], []
        for theta in theta_values:
            points, normals = [], []
            for index in range(sides):
                phi = 2.0 * math.pi * index / sides
                radial = tuple(lateral[k] * math.cos(phi) + (0.0, 0.0, 1.0)[k] * math.sin(phi) for k in range(3))
                points.append((cx - axis[0] * radius * math.cos(theta) + radial[0] * radius * math.sin(theta),
                               cy - axis[1] * radius * math.cos(theta) + radial[1] * radius * math.sin(theta),
                               cz + radial[2] * radius * math.sin(theta)))
                normals.append(tuple(-axis[k] * math.cos(theta) + radial[k] * math.sin(theta) for k in range(3)))
            ring_points.append(points)
            ring_normals.append(normals)
        for index in range(sides):
            nxt = (index + 1) % sides
            self._triangle(material, apex, ring_points[0][nxt], ring_points[0][index],
                           vertex_normals=(apex_normal, ring_normals[0][nxt], ring_normals[0][index]))
        for ring_index in range(len(ring_points) - 1):
            lower, upper = ring_points[ring_index], ring_points[ring_index + 1]
            lower_normals, upper_normals = ring_normals[ring_index], ring_normals[ring_index + 1]
            for index in range(sides):
                nxt = (index + 1) % sides
                self.quad(material,
                          (lower[index], lower[nxt], upper[nxt], upper[index]),
                          normal_points=(lower_normals[index], lower_normals[nxt], upper_normals[nxt], upper_normals[index]))
        self.component_counts[component] += 1

    def add_light(self, position_m, spacing_tag, yaw_deg=0.0):
        self.lights.append({
            "position_m": [round(float(value), 3) for value in position_m],
            "id": spacing_tag,
            "yaw_deg": round(float(yaw_deg), 2),
        })

    def check_corridor_fixture(self, fixture_id, lateral_center, half_lateral, z_min, z_max, envelope, corridor_width):
        lateral_half = envelope["width"] / 2.0
        flight_bottom = envelope["center_height"] - envelope["height"] / 2.0
        flight_top = envelope["center_height"] + envelope["height"] / 2.0
        fixture_min = lateral_center - half_lateral
        fixture_max = lateral_center + half_lateral
        lateral_overlap = fixture_min < lateral_half and fixture_max > -lateral_half
        vertical_overlap = z_min < flight_top and z_max > flight_bottom
        lateral_gap = max(0.0, -lateral_half - fixture_max, fixture_min - lateral_half)
        vertical_gap = max(0.0, flight_bottom - z_max, z_min - flight_top)
        if lateral_overlap and vertical_overlap:
            self.clearance_violations.append({"fixture": fixture_id, "scope": "corridor", "lateral_overlap": True, "vertical_overlap": True})
        elif vertical_overlap:
            self.lateral_margins.append(lateral_gap)
        elif lateral_overlap:
            self.vertical_margins.append(vertical_gap)
        else:
            self.lateral_margins.append(lateral_gap)
            self.vertical_margins.append(vertical_gap)
        self.clearance_records.append({"fixture": fixture_id, "scope": "corridor", "lateral_gap_m": round(lateral_gap, 3), "vertical_gap_m": round(vertical_gap, 3)})

    def check_room_fixture(self, fixture_id, center, size, envelope):
        half_env = (envelope["width"] / 2.0, envelope["width"] / 2.0, envelope["height"] / 2.0)
        flight_center = (0.0, 0.0, envelope["center_height"])
        half = (size[0] / 2.0, size[1] / 2.0, size[2] / 2.0)
        gaps = [max(0.0, abs(center[i] - flight_center[i]) - half[i] - half_env[i]) for i in range(3)]
        overlaps = [gaps[i] == 0.0 for i in range(3)]
        if all(overlaps):
            self.clearance_violations.append({"fixture": fixture_id, "scope": "junction", "overlap_axes": ["x", "y", "z"]})
        else:
            lateral_gap = max(gaps[0], gaps[1])
            if lateral_gap > 0.0:
                self.lateral_margins.append(lateral_gap)
            if gaps[2] > 0.0 and lateral_gap == 0.0:
                self.vertical_margins.append(gaps[2])
        self.clearance_records.append({"fixture": fixture_id, "scope": "junction", "axis_gaps_m": [round(gap, 3) for gap in gaps]})

    def check_room_lateral_fixture(self, fixture_id, center, radius, envelope):
        """Check wall-side services by horizontal clearance without inventing vertical margin."""
        half_env = envelope["width"] / 2.0
        dx = max(0.0, abs(center[0]) - half_env - radius)
        dy = max(0.0, abs(center[1]) - half_env - radius)
        lateral_gap = math.hypot(dx, dy)
        flight_bottom = envelope["center_height"] - envelope["height"] / 2.0
        flight_top = envelope["center_height"] + envelope["height"] / 2.0
        vertical_overlap = center[2] + radius > flight_bottom and center[2] - radius < flight_top
        if lateral_gap < 1e-6 and vertical_overlap:
            self.clearance_violations.append({"fixture": fixture_id, "scope": "junction_service", "lateral_overlap": True, "vertical_overlap": True})
        else:
            self.lateral_margins.append(lateral_gap)
        self.clearance_records.append({"fixture": fixture_id, "scope": "junction_service", "lateral_gap_m": round(lateral_gap, 3)})


def _local_world(start, yaw, along, lateral, z):
    return (
        start[0] + along * math.cos(yaw) - lateral * math.sin(yaw),
        start[1] + along * math.sin(yaw) + lateral * math.cos(yaw),
        z,
    )


def _route_port_into_return_pier(port, inward, side, entry: float = 0.10, radius: float = 0.35, samples: int = 12):
    """Smoothly turn a wall-lane service run into its adjacent structural return."""
    x0, y0, z0 = port
    start = (x0 + inward[0] * entry, y0 + inward[1] * entry, z0)
    end = (start[0] + inward[0] * radius + side[0] * radius,
           start[1] + inward[1] * radius + side[1] * radius,
           z0)
    control_scale = 0.55228475 * radius
    first_control = (start[0] + inward[0] * control_scale,
                     start[1] + inward[1] * control_scale, z0)
    second_control = (end[0] - side[0] * control_scale,
                      end[1] - side[1] * control_scale, z0)
    points = [port, start]
    for index in range(1, samples + 1):
        t = index / samples
        one_minus_t = 1.0 - t
        weights = (one_minus_t ** 3,
                   3.0 * one_minus_t ** 2 * t,
                   3.0 * one_minus_t * t ** 2,
                   t ** 3)
        coordinates = []
        for axis in range(3):
            coordinates.append(
                weights[0] * start[axis]
                + weights[1] * first_control[axis]
                + weights[2] * second_control[axis]
                + weights[3] * end[axis]
            )
        points.append(tuple(coordinates))
    return points


def _service_lane_offset(direction: str, width: float) -> float:
    """Use one world-side convention so a pipe aligns through all port orientations."""
    if direction not in VECTORS:
        raise ValueError(f"unknown service direction: {direction}")
    side_sign = 1.0 if direction in ("east", "north") else -1.0
    wall_clearance = 0.20 if width < 3.5 else 0.25
    return side_sign * (width / 2.0 - wall_clearance)


def _route_points_to_hub(edge_start, direction, opening_width, hub_xy, hub_inset, z, lane_offset=None):
    """Route an opening-side service lane into a shared corner manifold."""
    vx, vy = VECTORS[direction]
    yaw = math.atan2(vy, vx)
    lane = _service_lane_offset(direction, opening_width) if lane_offset is None else lane_offset
    lateral_axis = (-math.sin(yaw), math.cos(yaw))
    start = _local_world(edge_start, yaw, 0.0, lane, z)
    target = (hub_xy[0] + vx * hub_inset, hub_xy[1] + vy * hub_inset)
    delta_x, delta_y = target[0] - edge_start[0], target[1] - edge_start[1]
    along_end = delta_x * vx + delta_y * vy
    lateral_end = delta_x * lateral_axis[0] + delta_y * lateral_axis[1]
    points = [start]
    if abs(lateral_end - lane) > 1e-3:
        transition_along = -min(0.85, abs(along_end) * 0.42)
        transition_fraction = min(1.0, abs(transition_along / along_end))
        transition_lane = lane + (lateral_end - lane) * transition_fraction
        points.append(_local_world(edge_start, yaw, transition_along, transition_lane, z))
    points.append((target[0], target[1], z))
    return points


def _box_local(scene, start, yaw, along, lateral, z, size, material, component, envelope=None, envelope_width=None):
    center = _local_world(start, yaw, along, lateral, z)
    scene.box(center, size, material, component, yaw)
    if envelope is not None:
        scene.check_corridor_fixture(
            f"{component}_{scene.component_counts[component]}", lateral, size[1] / 2.0,
            z - size[2] / 2.0, z + size[2] / 2.0, envelope, envelope_width,
        )
    return center


def _pipe_local(
    scene,
    start,
    yaw,
    along0,
    along1,
    lateral,
    z,
    radius,
    material,
    envelope=None,
    envelope_width=None,
    fixture_id="pipe",
    cap_start=True,
    cap_end=True,
):
    a = _local_world(start, yaw, along0, lateral, z)
    b = _local_world(start, yaw, along1, lateral, z)
    scene.cylinder(a, b, radius, material, "pipe_run", cap_start=cap_start, cap_end=cap_end)
    # Long painted service lines use small metallic sleeve couplings at practical intervals.
    # These collars are separate geometry rather than a texture stripe.
    if material in {"Pipe_WaterBlue", "Pipe_FireRed", "Pipe_ServiceGreen"}:
        position = along0 + 2.75
        while position < along1 - 0.25:
            collar_a = _local_world(start, yaw, position - 0.025, lateral, z)
            collar_b = _local_world(start, yaw, position + 0.025, lateral, z)
            scene.cylinder(collar_a, collar_b, radius + 0.009, "Pipe_Coupling", "pipe_coupling")
            position += 5.5
    if envelope is not None:
        scene.check_corridor_fixture(f"{fixture_id}_{scene.component_counts['pipe_run']}", lateral, radius, z - radius, z + radius, envelope, envelope_width)


def _add_cable_tray_run(
    scene,
    start,
    yaw,
    along0,
    along1,
    lateral,
    z,
    tray_width,
    component="cable_tray",
    cap_start=True,
    cap_end=True,
):
    """Build an open steel ladder tray with visible individual cable bundles."""
    length = along1 - along0
    center_s = (along0 + along1) / 2.0
    rail_offset = tray_width / 2.0 - 0.025
    for side in (-1.0, 1.0):
        _box_local(scene, start, yaw, center_s, lateral + side * rail_offset, z,
                   (length, 0.035, 0.045), "Coated_Steel", component)
    rung_spacing = 0.42
    rung = 0.10
    while rung < length - 0.04:
        _box_local(scene, start, yaw, along0 + rung, lateral, z - 0.025,
                   (0.035, tray_width - 0.03, 0.025), "Coated_Steel", component)
        rung += rung_spacing
    cable_count = 3 if tray_width >= 0.5 else 2
    for index in range(cable_count):
        cable_lateral = lateral + (index - (cable_count - 1) / 2.0) * 0.105
        _pipe_local(
            scene,
            start,
            yaw,
            along0,
            along1,
            cable_lateral,
            z + 0.025,
            0.027,
            "Cable_Jacket",
            fixture_id=f"{component}_bundle",
            cap_start=cap_start,
            cap_end=cap_end,
        )


def _add_cable_tray_path(scene, points, tray_width, component="junction_cable_tray"):
    """Join short tray sections into one routed path; a junction box covers the shared end."""
    for start, end in zip(points, points[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 1e-5:
            continue
        yaw = math.atan2(dy, dx)
        _add_cable_tray_run(
            scene,
            start[:2],
            yaw,
            0.0,
            length,
            0.0,
            start[2],
            tray_width,
            component,
            cap_start=False,
            cap_end=False,
        )


def _add_junction_cable_bundle(
    scene,
    points,
    component="junction_cable_conduit",
    cable_count=3,
    cap_start=False,
    cap_end=False,
):
    """Route three narrow insulated cables through junction bends without tray slabs."""
    points = [tuple(float(value) for value in point) for point in points]
    if cable_count < 1:
        raise ValueError("junction cable bundle must contain at least one cable")
    lateral_offsets = [(index - (cable_count - 1) / 2.0) * 0.105 for index in range(cable_count)]
    for index, lateral_offset in enumerate(lateral_offsets):
        shifted = []
        for point_index, point in enumerate(points):
            if point_index == 0:
                dx = points[1][0] - point[0]
                dy = points[1][1] - point[1]
            elif point_index == len(points) - 1:
                dx = point[0] - points[point_index - 1][0]
                dy = point[1] - points[point_index - 1][1]
            else:
                dx = points[point_index + 1][0] - points[point_index - 1][0]
                dy = points[point_index + 1][1] - points[point_index - 1][1]
            length = math.hypot(dx, dy)
            if length < 1e-6:
                raise ValueError(f"degenerate junction cable path for {component}")
            nx, ny = -dy / length, dx / length
            shifted.append((point[0] + nx * lateral_offset, point[1] + ny * lateral_offset, point[2]))
        scene.tube_path(
            shifted,
            0.022,
            "Cable_Jacket",
            component,
            sides=48,
            cap_start=cap_start,
            cap_end=cap_end,
        )


def _room_fixture_box(scene, node, center, size, material, component, envelope):
    world_center = (node[0] + center[0], node[1] + center[1], center[2])
    scene.box(world_center, size, material, component)
    scene.check_room_fixture(f"{component}_{scene.component_counts[component]}", center, size, envelope)


def _add_inspection_hatch(scene: _SceneMesh, node, direction: str, room_width: float) -> None:
    """Add a small, wall-flush access panel instead of a box-like wall obstacle."""
    sign = 1.0 if direction in {"east", "north"} else -1.0
    half = room_width / 2.0
    normal = sign * (half - 0.015)
    panel_normal = sign * (half - 0.065)
    along = -1.35 if direction in {"east", "west"} else 1.35
    is_vertical_wall = direction in {"east", "west"}

    def wall_box(n, s, z, width_along, depth, height, material, component):
        if is_vertical_wall:
            center = (n, along + s, z)
            size = (depth, width_along, height)
        else:
            center = (along + s, n, z)
            size = (width_along, depth, height)
        scene.box((node[0] + center[0], node[1] + center[1], center[2]), size, material, component)

    # Recessed, framed 0.62 x 0.80 m panel with simple hinges and a visible latch.
    wall_box(normal, -0.34, 1.25, 0.04, 0.03, 0.90, "Coated_Steel", "inspection_hatch_frame")
    wall_box(normal, 0.34, 1.25, 0.04, 0.03, 0.90, "Coated_Steel", "inspection_hatch_frame")
    wall_box(normal, 0.0, 1.68, 0.72, 0.03, 0.04, "Coated_Steel", "inspection_hatch_frame")
    wall_box(normal, 0.0, 0.82, 0.72, 0.03, 0.04, "Coated_Steel", "inspection_hatch_frame")
    wall_box(panel_normal, 0.0, 1.25, 0.62, 0.03, 0.80, "Concrete_Trim", "inspection_hatch_panel")
    wall_box(sign * (half - 0.045), -0.23, 1.25, 0.025, 0.025, 0.15, "Safety_Marking", "inspection_hatch_latch")
    for z in (0.96, 1.54):
        wall_box(sign * (half - 0.045), 0.32, z, 0.035, 0.035, 0.10, "Pipe_Coupling", "inspection_hatch_hinge")


def _add_branch_terminal_bulkhead(scene: _SceneMesh, branch: dict, wall_thickness: float):
    """Close each service spur with a plain bulkhead behind visible capped runs."""
    start, end = branch["start_m"], branch["end_m"]
    length = branch["length_m"]
    width, height = branch["width_m"], branch["height_m"]
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    wall_center_along = length + wall_thickness / 2.0
    center = _local_world(start, yaw, wall_center_along, 0.0, height / 2.0)
    scene.box(center, (wall_thickness, width, height), "Concrete_WarmGrey",
              "branch_terminal_bulkhead", yaw)


def _add_concrete_shell(scene: _SceneMesh, layout: dict):
    dims = layout["dimensions_m"]
    thickness = dims["shell_thickness"]
    wall_t = dims["junction_wall_thickness"]
    room_w = dims["junction_width"]
    room_h = dims["junction_height"]
    directions: dict[str, dict[str, tuple[float, float]]] = {node["id"]: {} for node in layout["nodes"]}
    node_by_id = {node["id"]: tuple(node["position_m"]) for node in layout["nodes"]}
    for edge in layout["edges"]:
        a, b = edge["from"], edge["to"]
        direction = edge["direction"]
        directions[a][direction] = (edge["width_m"], edge["height_m"])
        directions[b][{"east": "west", "north": "south"}[direction]] = (edge["width_m"], edge["height_m"])
    for branch in layout["branches"]:
        directions[branch["from_node"]][branch["direction"]] = (branch["width_m"], branch["height_m"])

    for edge in layout["edges"]:
        start, end = edge["start_m"], edge["end_m"]
        length, width, height = edge["length_m"], edge["width_m"], edge["height_m"]
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
        outside_width = width + 2.0 * thickness
        scene.box((center[0], center[1], -thickness / 2.0), (length, outside_width, thickness), "Concrete_WarmGrey", "corridor_shell", yaw)
        scene.box((center[0], center[1], height + thickness / 2.0), (length, outside_width, thickness), "Concrete_WarmGrey", "corridor_shell", yaw)
        for side in (-1.0, 1.0):
            wall_center = (center[0] - side * math.sin(yaw) * (width / 2.0 + thickness / 2.0), center[1] + side * math.cos(yaw) * (width / 2.0 + thickness / 2.0), height / 2.0)
            scene.box(wall_center, (length, thickness, height), "Concrete_WarmGrey", "corridor_shell", yaw)

    for branch in layout["branches"]:
        start, end = branch["start_m"], branch["end_m"]
        length, width, height = branch["length_m"], branch["width_m"], branch["height_m"]
        yaw = math.atan2(end[1] - start[1], end[0] - start[0])
        center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
        outside_width = width + 2.0 * thickness
        scene.box((center[0], center[1], -thickness / 2.0), (length, outside_width, thickness), "Concrete_WarmGrey", "corridor_shell", yaw)
        scene.box((center[0], center[1], height + thickness / 2.0), (length, outside_width, thickness), "Concrete_WarmGrey", "corridor_shell", yaw)
        for side in (-1.0, 1.0):
            wall_center = (center[0] - side * math.sin(yaw) * (width / 2.0 + thickness / 2.0), center[1] + side * math.cos(yaw) * (width / 2.0 + thickness / 2.0), height / 2.0)
            scene.box(wall_center, (length, thickness, height), "Concrete_WarmGrey", "corridor_shell", yaw)
        _add_branch_terminal_bulkhead(scene, branch, thickness)

    for node in layout["nodes"]:
        node_id = node["id"]
        x, y = node_by_id[node_id]
        open_dirs = directions[node_id]
        scene.box((x, y, -thickness / 2.0), (room_w + 2 * thickness, room_w + 2 * thickness, thickness), "Concrete_WarmGrey", "junction_shell")
        scene.box((x, y, room_h + thickness / 2.0), (room_w + 2 * thickness, room_w + 2 * thickness, thickness), "Concrete_WarmGrey", "junction_shell")
        for direction in ("east", "west", "north", "south"):
            opening = open_dirs.get(direction)
            if direction in ("east", "west"):
                wall_center = (x + (1 if direction == "east" else -1) * (room_w / 2.0 + wall_t / 2.0), y, room_h / 2.0)
                if opening is None:
                    scene.box(wall_center, (wall_t, room_w, room_h), "Concrete_WarmGrey", "junction_shell")
                else:
                    opening_width, opening_height = opening
                    pier = (room_w - opening_width) / 2.0
                    for side in (-1.0, 1.0):
                        scene.box((wall_center[0], y + side * (opening_width / 2.0 + pier / 2.0), room_h / 2.0), (wall_t, pier, room_h), "Concrete_WarmGrey", "junction_shell")
                        if pier > 0.30:
                            return_depth = 0.75
                            return_center_x = x + (1 if direction == "east" else -1) * (room_w / 2.0 - return_depth / 2.0)
                            return_center_y = y + side * (opening_width / 2.0 + pier / 2.0)
                            scene.box((return_center_x, return_center_y, room_h / 2.0),
                                      (return_depth, pier, room_h),
                                      "Concrete_WarmGrey", "portal_structural_return")
                            scene.service_routing["portal_return_pier_count"] += 1
                    header_h = room_h - min(opening_height, room_h - 0.25)
                    if header_h > 0.0:
                        scene.box((wall_center[0], y, room_h - header_h / 2.0), (wall_t, opening_width, header_h), "Concrete_Trim", "junction_shell")
            else:
                wall_center = (x, y + (1 if direction == "north" else -1) * (room_w / 2.0 + wall_t / 2.0), room_h / 2.0)
                if opening is None:
                    scene.box(wall_center, (room_w, wall_t, room_h), "Concrete_WarmGrey", "junction_shell")
                else:
                    opening_width, opening_height = opening
                    pier = (room_w - opening_width) / 2.0
                    for side in (-1.0, 1.0):
                        scene.box((x + side * (opening_width / 2.0 + pier / 2.0), wall_center[1], room_h / 2.0), (pier, wall_t, room_h), "Concrete_WarmGrey", "junction_shell")
                        if pier > 0.30:
                            return_depth = 0.75
                            return_center_x = x + side * (opening_width / 2.0 + pier / 2.0)
                            return_center_y = y + (1 if direction == "north" else -1) * (room_w / 2.0 - return_depth / 2.0)
                            scene.box((return_center_x, return_center_y, room_h / 2.0),
                                      (pier, return_depth, room_h),
                                      "Concrete_WarmGrey", "portal_structural_return")
                            scene.service_routing["portal_return_pier_count"] += 1
                    header_h = room_h - min(opening_height, room_h - 0.25)
                    if header_h > 0.0:
                        scene.box((x, wall_center[1], room_h - header_h / 2.0), (opening_width, wall_t, header_h), "Concrete_Trim", "junction_shell")

        # Keep the junction walls visually quiet in this navigation preview.
        # A recessed access hatch read as a freestanding cabinet from the
        # earlier AirSim camera angle and distracted from the pipe routing.

    return scene, directions


def _add_corridor_services(scene: _SceneMesh, path_id: str, start, end, width, height, envelope, edge_spacing, node_id=None, direction=None, terminate_at_end=False):
    length = math.dist(start, end)
    yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    direction = direction or ("east" if abs(math.cos(yaw)) > abs(math.sin(yaw)) and math.cos(yaw) >= 0 else
                              "west" if abs(math.cos(yaw)) > abs(math.sin(yaw)) else
                              "north" if math.sin(yaw) >= 0 else "south")
    pipe_side = _service_lane_offset(direction, width)
    tray_side = -pipe_side
    pipe_specs = ((0.92, 0.09, "Pipe_WaterBlue"), (1.43, 0.08, "Pipe_FireRed"), (1.94, 0.06, "Pipe_ServiceGreen"), (2.38, 0.045, "Pipe_Insulation"))
    edge_start = start
    pipe_start = 0.0
    # Connected runs meet junction runs at the same opening plane. Leaving
    # both ends capped or overlapping their cylinders caused the dark, jagged
    # seams seen in the AirSim screenshots.
    # At a closed branch, carry the service lines through the bulkhead's near
    # face and stop them inside the concrete. This hides the cut cross-section
    # and reads as a real wall penetration instead of a pipe ending in space.
    pipe_end = length + 0.20 if terminate_at_end else length
    for index, (z, radius, material) in enumerate(pipe_specs):
        _pipe_local(
            scene,
            edge_start,
            yaw,
            pipe_start,
            pipe_end,
            pipe_side,
            z,
            radius,
            material,
            envelope,
            width,
            f"{path_id}_pipe{index}",
            cap_start=False,
            cap_end=False,
        )
        if terminate_at_end:
            # Add a short sleeve across the wall plane. The service pipe
            # continues underneath it into the bulkhead, so neither piece has
            # a visible open or tapered end in the corridor view.
            sleeve_start = _local_world(edge_start, yaw, length - 0.24, pipe_side, z)
            sleeve_end = _local_world(edge_start, yaw, length + 0.03, pipe_side, z)
            scene.cylinder(sleeve_start, sleeve_end, radius + 0.025,
                           "Pipe_Coupling", "branch_wall_sleeve",
                           sides=48, cap_start=False, cap_end=False)
            wall_face = _local_world(edge_start, yaw, length - 0.01, pipe_side, z)
            scene.annular_ring_on_axis(wall_face, yaw, radius + 0.035,
                                       max(0.20, radius + 0.11), "Service_Trim",
                                       "branch_wall_escutcheon")
            scene.service_routing["branch_wall_sleeve_count"] += 1
            scene.service_routing["branch_wall_penetration_count"] += 1
            scene.service_routing["branch_wall_escutcheon_count"] += 1
    tray_width = 0.48 if width < 3.5 else 0.62
    tray_center_z = min(height - 0.68, 2.78)
    tray_end = length + 0.15 if terminate_at_end else length
    _add_cable_tray_run(
        scene,
        edge_start,
        yaw,
        0.0,
        tray_end,
        tray_side,
        tray_center_z,
        tray_width,
        cap_start=False,
        cap_end=False,
    )
    support_spacing = edge_spacing["cable_support_spacing_m"]
    support_positions = []
    position = support_spacing / 2.0
    while position < length - 1.0:
        support_positions.append(position)
        wall_sign = 1.0 if pipe_side >= 0 else -1.0
        _box_local(scene, edge_start, yaw, position, wall_sign * (width / 2.0 - 0.045), 1.68,
                   (0.045, 0.055, 1.85), "Coated_Steel", "support_bracket", envelope, width)
        for pipe_z, pipe_radius, _ in pipe_specs:
            _box_local(scene, edge_start, yaw, position, pipe_side + wall_sign * 0.10,
                       pipe_z - pipe_radius - 0.015, (0.025, 0.20, 0.025), "Coated_Steel", "support_bracket", envelope, width)
        tray_wall_sign = 1.0 if tray_side > 0 else -1.0
        _box_local(scene, edge_start, yaw, position, tray_side + tray_wall_sign * 0.10,
                   tray_center_z + 0.10, (0.045, 0.22, 0.035), "Coated_Steel", "support_bracket", envelope, width)
        position += support_spacing
    joint_spacing = edge_spacing["structural_joint_spacing_m"]
    position = joint_spacing
    while position < length - 1.0:
        for side in (-1.0, 1.0):
            _box_local(scene, edge_start, yaw, position, side * (width / 2.0 - 0.002), height / 2.0,
                       (0.008, 0.008, height - 0.02), "Concrete_Joint", "structural_joint", envelope, width)
        _box_local(scene, edge_start, yaw, position, 0.0, 0.006,
                   (0.008, width, 0.008), "Concrete_Joint", "structural_joint", envelope, width)
        _box_local(scene, edge_start, yaw, position, 0.0, height - 0.006,
                   (0.008, width, 0.008), "Concrete_Joint", "structural_joint", envelope, width)
        position += joint_spacing
    # Drain channels follow the wall-side floor and do not cross the central flight lane.
    for side in (-1.0, 1.0):
        _box_local(scene, edge_start, yaw, length / 2.0, side * (width / 2.0 - 0.14), 0.035,
                   (length, 0.12, 0.07), "Coated_Steel", "drain_channel", envelope, width)

    light_spacing = edge_spacing["light_spacing_m"]
    light_positions = []
    position = light_spacing / 2.0
    while position < length - 1.0:
        fixture_center = _box_local(scene, edge_start, yaw, position, 0.0, height - 0.10,
                                    (0.82, 0.13, 0.045), "Light_Cover", "luminaire", envelope, width)
        scene.add_light((fixture_center[0], fixture_center[1], height - 0.46),
                        f"{path_id}_L{len(light_positions):02d}", math.degrees(yaw))
        light_positions.append(round(position, 3))
        position += light_spacing

    return {"support_positions_m": support_positions, "light_positions_m": light_positions, "joint_spacing_m": joint_spacing}


def _minimum_corner_tree(active_corners):
    """Return the fewest wall-perimeter segments that connect active port corners."""
    perimeter_edges = (
        ("NW", "NE"),
        ("NE", "SE"),
        ("SE", "SW"),
        ("SW", "NW"),
    )
    active = set(active_corners)
    if len(active) < 2:
        return []

    for edge_count in range(1, len(perimeter_edges) + 1):
        for mask in range(1 << len(perimeter_edges)):
            if mask.bit_count() != edge_count:
                continue
            chosen = [edge for index, edge in enumerate(perimeter_edges) if mask & (1 << index)]
            adjacency = {corner: set() for corner in ("NW", "NE", "SE", "SW")}
            for first, second in chosen:
                adjacency[first].add(second)
                adjacency[second].add(first)
            visited = {next(iter(active))}
            pending = list(visited)
            while pending:
                current = pending.pop()
                for neighbor in adjacency[current] - visited:
                    visited.add(neighbor)
                    pending.append(neighbor)
            if active <= visited:
                return chosen
    raise RuntimeError(f"could not connect junction port corners: {sorted(active)}")


def _add_junction_services(scene: _SceneMesh, node, open_dirs, envelope):
    x, y = node["position_m"]
    room_w, room_h = node["width_m"], node["height_m"]
    pipe_specs = ((0.92, 0.09, "Pipe_WaterBlue"), (1.43, 0.08, "Pipe_FireRed"),
                  (1.94, 0.06, "Pipe_ServiceGreen"), (2.38, 0.045, "Pipe_Insulation"))
    # Each corridor-side service turns into the structural return beside its
    # own portal. The return pier hides the route end; no exposed manifold or
    # pipe/cable crossbar is drawn across the junction's clear center.
    for direction in open_dirs:
        vx, vy = VECTORS[direction]
        edge_start = (x + vx * room_w / 2.0, y + vy * room_w / 2.0)
        opening_width, _ = open_dirs[direction]
        pipe_lane = _service_lane_offset(direction, opening_width)
        if abs(pipe_lane) != abs(_service_lane_offset(direction, max(width for width, _ in open_dirs.values()))):
            scene.service_routing["branch_lane_transition_count"] += 1
        yaw = math.atan2(vy, vx)
        lateral = (-math.sin(yaw), math.cos(yaw))
        inward = (-vx, -vy)
        pipe_side_sign = 1.0 if pipe_lane >= 0.0 else -1.0
        pipe_side = (lateral[0] * pipe_side_sign, lateral[1] * pipe_side_sign)
        cable_side = (-pipe_side[0], -pipe_side[1])

        for pipe_i, (z, radius, material) in enumerate(pipe_specs):
            port = _local_world(edge_start, yaw, 0.0, pipe_lane, z)
            points = _route_port_into_return_pier(
                port, inward, pipe_side, entry=0.10, radius=0.35, samples=12,
            )
            scene.tube_path(
                points,
                radius,
                material,
                "junction_pipe_run",
                sides=48,
                cap_start=False,
                # The end is buried inside the concrete portal return.
                cap_end=False,
            )
            # An open-ended coupling sleeve bridges the corridor and junction
            # runs without introducing a solid disk or dark hole at the portal.
            port_sleeve_start = _local_world(edge_start, yaw, -0.07, pipe_lane, z)
            port_sleeve_end = _local_world(edge_start, yaw, 0.07, pipe_lane, z)
            scene.cylinder(port_sleeve_start, port_sleeve_end, radius + 0.03,
                           "Pipe_Coupling", "junction_port_coupling",
                           sides=48, cap_start=False, cap_end=False)
            scene.service_routing["junction_port_coupling_count"] += 1
            for point_index, point in enumerate(points):
                scene.check_room_lateral_fixture(
                    f"{node['id']}_junction_pipe{pipe_i}_{direction}_p{point_index}",
                    (point[0] - x, point[1] - y, z), radius, envelope,
                )
        tray_lane = -_service_lane_offset(direction, opening_width)
        tray_port = _local_world(edge_start, yaw, 0.0, tray_lane, 2.80)
        # Keep the same three-conductor bundle at every port. Varying the count
        # by corridor width changed lateral offsets at the node and left visible
        # cable tips at the branch junction.
        cable_count = 3
        cable_points = _route_port_into_return_pier(
            tray_port, inward, cable_side, entry=0.10, radius=0.35, samples=12,
        )
        _add_junction_cable_bundle(
            scene,
            cable_points,
            cable_count=cable_count,
            cap_end=False,
        )
    scene.service_routing["junctions"].append({
        "node_id": node["id"],
        "open_ports": sorted(open_dirs),
        "pipe_fittings": 0,
        "routing": "separate_concealed_portal_return_runs",
        "pipe_perimeter_edges": [],
        "cable_perimeter_edges": [],
        "visible_center_crossbars": 0,
    })
    scene.service_routing["junction_service_return_count"] += len(open_dirs)
    scene.service_routing["junction_count"] = len(scene.service_routing["junctions"])

    # Two independently placed luminaires make room scale and height legible.
    for index, offset in enumerate((-1.8, 1.8)):
        light_center = (x + offset, y + 0.4, room_h - 0.10)
        scene.box(light_center, (0.82, 0.13, 0.045), "Light_Cover", "luminaire")
        scene.check_room_fixture(f"{node['id']}_light{index}", (offset, 0.4, room_h - 0.10), (0.82, 0.13, 0.045), envelope)
        scene.add_light((light_center[0], light_center[1], room_h - 0.46), f"{node['id']}_L{index}", 0.0)

    # The junction shell already places a small inspection hatch on a closed
    # wall. Do not add a fixed east-wall cabinet here: a junction may have an
    # open east port, in which case that cabinet blocks the corridor branch.



def _add_services(scene: _SceneMesh, layout: dict, directions: dict):
    spacing_meta = {"structural_joint": {}, "luminaire": {}}
    for edge in layout["edges"]:
        start, end = edge["start_m"], edge["end_m"]
        result = _add_corridor_services(scene, edge["id"], start, end, edge["width_m"], edge["height_m"], layout["flight_envelope_m"], edge, edge["from"], edge["direction"])
        spacing_meta["structural_joint"][edge["id"]] = result["joint_spacing_m"]
        spacing_meta["luminaire"][edge["id"]] = edge["light_spacing_m"]
    for branch in layout["branches"]:
        spacing = {
            "structural_joint_spacing_m": 5.4,
            "cable_support_spacing_m": 2.7,
            "light_spacing_m": 5.0,
        }
        _add_corridor_services(scene, branch["id"], branch["start_m"], branch["end_m"], branch["width_m"], branch["height_m"], layout["flight_envelope_m"], spacing, branch["from_node"], branch["direction"], terminate_at_end=True)
    for node in layout["nodes"]:
        _add_junction_services(scene, node, directions[node["id"]], layout["flight_envelope_m"])
    return spacing_meta


def _add_geometry(layout: dict) -> tuple[_SceneMesh, dict]:
    scene = _SceneMesh()
    scene, directions = _add_concrete_shell(scene, layout)
    spacing_meta = _add_services(scene, layout, directions)
    return scene, spacing_meta


def _build_glb(scene: _SceneMesh, concrete_albedo_png: bytes, concrete_normal_png: bytes, concrete_mr_png: bytes, signage_png: bytes) -> bytes:
    binary = bytearray()
    buffer_views = []
    accessors = []

    def append_buffer(data: bytes, target=None):
        while len(binary) % 4:
            binary.append(0)
        offset = len(binary)
        binary.extend(data)
        view = {"buffer": 0, "byteOffset": offset, "byteLength": len(data)}
        if target is not None:
            view["target"] = target
        buffer_views.append(view)
        return len(buffer_views) - 1

    def append_accessor(data: bytes, target, component_type, count, kind, minimum=None, maximum=None):
        view_index = append_buffer(data, target)
        accessor = {"bufferView": view_index, "componentType": component_type, "count": count, "type": kind}
        if minimum is not None:
            accessor["min"] = minimum
        if maximum is not None:
            accessor["max"] = maximum
        accessors.append(accessor)
        return len(accessors) - 1

    primitives = []
    for material in MATERIALS:
        name = material["name"]
        group = scene.groups.get(name)
        if not group or not group["indices"]:
            continue
        positions = group["positions"]
        normals = group["normals"]
        uvs = group["uvs"]
        indices = group["indices"]
        # Unreal's glTF importer treats source Y as its up axis. Export the
        # z-up gallery frame as a Y-up glTF frame. The negative UE Y term makes
        # this a proper rotation (determinant +1), preserving handedness and
        # keeping the imported mesh aligned with the AirSim world axes.
        exported_positions = [(vertex[0], vertex[2], -vertex[1]) for vertex in positions]
        exported_normals = [(vertex[0], vertex[2], -vertex[1]) for vertex in normals]
        position_flat = [value for vertex in exported_positions for value in vertex]
        normal_flat = [value for vertex in exported_normals for value in vertex]
        uv_flat = [value for vertex in uvs for value in vertex]
        position_accessor = append_accessor(
            struct.pack("<" + "f" * len(position_flat), *position_flat), 34962, 5126, len(positions), "VEC3",
            [min(vertex[axis] for vertex in exported_positions) for axis in range(3)],
            [max(vertex[axis] for vertex in exported_positions) for axis in range(3)],
        )
        normal_accessor = append_accessor(struct.pack("<" + "f" * len(normal_flat), *normal_flat), 34962, 5126, len(normals), "VEC3")
        uv_accessor = append_accessor(struct.pack("<" + "f" * len(uv_flat), *uv_flat), 34962, 5126, len(uvs), "VEC2")
        index_accessor = append_accessor(struct.pack("<" + "I" * len(indices), *indices), 34963, 5125, len(indices), "SCALAR")
        primitives.append(
            {
                "attributes": {"POSITION": position_accessor, "NORMAL": normal_accessor, "TEXCOORD_0": uv_accessor},
                "indices": index_accessor,
                "material": MATERIAL_INDEX[name],
                "mode": 4,
            }
        )

    concrete_albedo_view = append_buffer(concrete_albedo_png)
    signage_view = append_buffer(signage_png)
    concrete_normal_view = append_buffer(concrete_normal_png)
    concrete_mr_view = append_buffer(concrete_mr_png)
    materials = []
    for source in MATERIALS:
        material = {
            "name": source["name"],
            "doubleSided": True,
            "pbrMetallicRoughness": {
                "baseColorFactor": source["baseColorFactor"],
                "metallicFactor": source["metallicFactor"],
                "roughnessFactor": source["roughnessFactor"],
            },
        }
        if source.get("emissiveFactor"):
            material["emissiveFactor"] = source["emissiveFactor"]
        if source.get("texture") == "concrete":
            material["pbrMetallicRoughness"]["baseColorTexture"] = {"index": 0}
            material["pbrMetallicRoughness"]["metallicRoughnessTexture"] = {"index": 3}
            material["normalTexture"] = {"index": 2, "scale": 0.10}
        elif source.get("texture") == "signage":
            material["pbrMetallicRoughness"]["baseColorTexture"] = {"index": 1}
        materials.append(material)

    json_doc = {
        "asset": {
            "version": "2.0",
            "generator": "utility_gallery_procedural_builder",
            "extras": {"coordinate_convention": "glTF Y-up: (X, Y, Z) = (UE X, UE Z, -UE Y); proper axis rotation"},
        },
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"name": "UtilityGalleryPreview", "mesh": 0}],
        "meshes": [{"name": "UtilityGalleryPreviewMesh", "primitives": primitives}],
        "materials": materials,
        "textures": [
            {"sampler": 0, "source": 0},
            {"sampler": 0, "source": 1},
            {"sampler": 0, "source": 2},
            {"sampler": 0, "source": 3},
        ],
        "samplers": [{"magFilter": 9729, "minFilter": 9987, "wrapS": 33648, "wrapT": 33648}],
        "images": [
            {"name": "Concrete_Albedo", "bufferView": concrete_albedo_view, "mimeType": "image/png"},
            {"name": "UtilityGallery_SignageAtlas", "bufferView": signage_view, "mimeType": "image/png"},
            {"name": "Concrete_Normal", "bufferView": concrete_normal_view, "mimeType": "image/png"},
            {"name": "Concrete_Roughness", "bufferView": concrete_mr_view, "mimeType": "image/png"},
        ],
        "bufferViews": buffer_views,
        "accessors": accessors,
        "buffers": [{"byteLength": len(binary)}],
    }
    json_bytes = json.dumps(json_doc, separators=(",", ":")).encode("utf-8")
    json_bytes += b" " * ((4 - len(json_bytes) % 4) % 4)
    while len(binary) % 4:
        binary.append(0)
    total_length = 12 + 8 + len(json_bytes) + 8 + len(binary)
    return b"".join(
        (
            struct.pack("<4sII", b"glTF", 2, total_length),
            struct.pack("<I4s", len(json_bytes), b"JSON"),
            json_bytes,
            struct.pack("<I4s", len(binary), b"BIN\0"),
            bytes(binary),
        )
    )


def generate_preview(
    glb_path: str | Path = DEFAULT_GLB,
    metadata_path: str | Path = DEFAULT_METADATA,
    *,
    config: dict | str | Path | None = None,
    texture_dir: str | Path | None = None,
) -> dict:
    """Write the gallery GLB and a human-readable geometry/clearance manifest."""
    glb_path, metadata_path = Path(glb_path), Path(metadata_path)
    glb_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    layout = build_layout(config)
    scene, spacing_meta = _add_geometry(layout)
    concrete_albedo_png, concrete_normal_png, concrete_mr_png = _concrete_material_textures(texture_dir)
    signage_png = _signage_atlas_png()
    glb = _build_glb(scene, concrete_albedo_png, concrete_normal_png, concrete_mr_png, signage_png)
    glb_path.write_bytes(glb)
    (glb_path.parent / "concrete_albedo.png").write_bytes(concrete_albedo_png)
    (glb_path.parent / "concrete_normal.png").write_bytes(concrete_normal_png)
    (glb_path.parent / "concrete_metallic_roughness.png").write_bytes(concrete_mr_png)
    (glb_path.parent / "utility_gallery_signage_atlas.png").write_bytes(signage_png)
    lateral_margin = min(scene.lateral_margins, default=0.0)
    vertical_margin = min(scene.vertical_margins, default=0.0)
    metadata = {
        "status": "utility_gallery_geometry_generated",
        "glb_path": str(glb_path),
        "glb_sha256": hashlib.sha256(glb).hexdigest(),
        "units": "meters",
        "bounds_m": {"min": [round(value, 3) for value in scene.bounds_min], "max": [round(value, 3) for value in scene.bounds_max]},
        "triangle_count": scene.triangle_count,
        "material_names": [material["name"] for material in MATERIALS if scene.groups.get(material["name"])],
        "component_counts": dict(sorted(scene.component_counts.items())),
        "layout_summary": {"node_count": len(layout["nodes"]), "edge_count": len(layout["edges"]), "branch_count": len(layout["branches"]), "cycle_rank": len(layout["edges"]) - len(layout["nodes"]) + 1},
        "dimensions_m": layout["dimensions_m"],
        "flight_envelope_m": layout["flight_envelope_m"],
        "clearance": {
            "violations": scene.clearance_violations,
            "minimum_lateral_margin_m": round(lateral_margin, 3),
            "minimum_vertical_margin_m": round(vertical_margin, 3),
            "fixture_count_checked": len(scene.clearance_records),
        },
        "edge_fixture_spacing_m": spacing_meta,
        "lights": scene.lights,
        "service_routing": scene.service_routing,
        "textures": {
            "concrete_albedo_sha256": hashlib.sha256(concrete_albedo_png).hexdigest(),
            "concrete_normal_sha256": hashlib.sha256(concrete_normal_png).hexdigest(),
            "concrete_metallic_roughness_sha256": hashlib.sha256(concrete_mr_png).hexdigest(),
            "signage_atlas_sha256": hashlib.sha256(signage_png).hexdigest(),
            "mode": "procedural" if texture_dir is None else "external",
            "concrete_texture_source": "deterministic code-generated matte PBR" if texture_dir is None else "user-supplied atlas; crop (340,150)-(852,662); redistribution rights must be checked separately",
            "concrete_uv_repeat_per_meter": 0.18,
        },
        "source_assets_used": [] if texture_dir is None else [str(Path(texture_dir) / name) for name in TEXTURE_FILES],
        "player_start_m": [*layout["nodes"][0]["position_m"], 1.25],
        "evidence_boundary": "generated geometry only; Unreal import, collision queries, sensors and flight are not validated",
    }
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    return metadata


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--config", type=Path, help="layout JSON; defaults to adjacent settings/utility_gallery.json")
    parser.add_argument("--texture-dir", type=Path,
                        help="optional licensed external atlas directory; never searched automatically")
    args = parser.parse_args(argv)
    result = generate_preview(args.output_dir / "utility_gallery_preview.glb",
                              args.output_dir / "utility_gallery_geometry.json",
                              config=args.config, texture_dir=args.texture_dir)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
