#!/usr/bin/env python3
"""Generate local-space OBJ meshes for the approximate camera shell and LiDAR mount."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

DEFAULT_OUTPUT = Path(__file__).resolve().parents[3] / 'build/airsimx/models'


class ObjWriter:
    def __init__(self, path: Path, materials: dict[str, tuple[float, float, float]]):
        self.path = path
        self.materials = materials
        self.vertices: list[tuple[float, float, float]] = []
        self.faces: list[tuple[str, list[int]]] = []
        self.material = next(iter(materials))

    def use(self, material: str) -> None:
        if material not in self.materials:
            raise ValueError(f'unknown material: {material}')
        self.material = material

    def _vertex(self, x: float, y: float, z: float) -> int:
        self.vertices.append((x, y, z))
        return len(self.vertices)

    def box(self, center: tuple[float, float, float], size: tuple[float, float, float]) -> None:
        cx, cy, cz = center
        hx, hy, hz = (v / 2.0 for v in size)
        ids = [
            self._vertex(cx - hx, cy - hy, cz - hz),
            self._vertex(cx + hx, cy - hy, cz - hz),
            self._vertex(cx + hx, cy + hy, cz - hz),
            self._vertex(cx - hx, cy + hy, cz - hz),
            self._vertex(cx - hx, cy - hy, cz + hz),
            self._vertex(cx + hx, cy - hy, cz + hz),
            self._vertex(cx + hx, cy + hy, cz + hz),
            self._vertex(cx - hx, cy + hy, cz + hz),
        ]
        for face in (
            (ids[1], ids[0], ids[3], ids[2]),
            (ids[4], ids[5], ids[6], ids[7]),
            (ids[0], ids[1], ids[5], ids[4]),
            (ids[2], ids[3], ids[7], ids[6]),
            (ids[1], ids[2], ids[6], ids[5]),
            (ids[3], ids[0], ids[4], ids[7]),
        ):
            self.faces.append((self.material, list(face)))

    def cylinder_x(self, x0: float, x1: float, cy: float, cz: float, radius: float, sides: int = 24) -> None:
        start = []
        end = []
        for i in range(sides):
            a = 2.0 * math.pi * i / sides
            y = cy + radius * math.cos(a)
            z = cz + radius * math.sin(a)
            start.append(self._vertex(x0, y, z))
            end.append(self._vertex(x1, y, z))
        for i in range(sides):
            j = (i + 1) % sides
            self.faces.append((self.material, [start[i], start[j], end[j], end[i]]))
        self.faces.append((self.material, list(reversed(start))))
        self.faces.append((self.material, end))

    def cylinder_z(self, cx: float, cy: float, z0: float, z1: float, radius: float, sides: int = 20) -> None:
        bottom = []
        top = []
        for i in range(sides):
            a = 2.0 * math.pi * i / sides
            x = cx + radius * math.cos(a)
            y = cy + radius * math.sin(a)
            bottom.append(self._vertex(x, y, z0))
            top.append(self._vertex(x, y, z1))
        for i in range(sides):
            j = (i + 1) % sides
            self.faces.append((self.material, [bottom[i], bottom[j], top[j], top[i]]))
        self.faces.append((self.material, list(reversed(bottom))))
        self.faces.append((self.material, top))

    def beam_xy(self, start: tuple[float, float], end: tuple[float, float], width: float,
                height: float, z_center: float) -> None:
        sx, sy = start
        ex, ey = end
        dx, dy = ex - sx, ey - sy
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            raise ValueError('beam endpoints must differ')
        px, py = -dy / length, dx / length
        half = width / 2.0
        corners = [
            (sx - px * half, sy - py * half),
            (ex - px * half, ey - py * half),
            (ex + px * half, ey + py * half),
            (sx + px * half, sy + py * half),
        ]
        z0, z1 = z_center - height / 2.0, z_center + height / 2.0
        ids = [self._vertex(x, y, z) for z in (z0, z1) for x, y in corners]
        for face in (
            (ids[1], ids[0], ids[3], ids[2]),
            (ids[4], ids[5], ids[6], ids[7]),
            (ids[0], ids[1], ids[5], ids[4]),
            (ids[2], ids[3], ids[7], ids[6]),
            (ids[1], ids[2], ids[6], ids[5]),
            (ids[3], ids[0], ids[4], ids[7]),
        ):
            self.faces.append((self.material, list(face)))

    def tube_between_xy(self, start: tuple[float, float], end: tuple[float, float],
                        width: float, height: float, z_center: float,
                        sides: int = 10) -> None:
        """Create a low-poly carbon-tube arm between XY points."""
        if sides < 6:
            raise ValueError('tube cross-section needs at least six sides')
        sx, sy = start
        ex, ey = end
        dx, dy = ex - sx, ey - sy
        length = math.hypot(dx, dy)
        if length <= 1e-6:
            raise ValueError('tube endpoints must differ')
        px, py = -dy / length, dx / length
        start_ring = []
        end_ring = []
        for index in range(sides):
            angle = 2.0 * math.pi * index / sides
            offset_xy = math.cos(angle) * width / 2.0
            offset_z = math.sin(angle) * height / 2.0
            start_ring.append(self._vertex(
                sx + px * offset_xy, sy + py * offset_xy, z_center + offset_z
            ))
            end_ring.append(self._vertex(
                ex + px * offset_xy, ey + py * offset_xy, z_center + offset_z
            ))
        for index in range(sides):
            next_index = (index + 1) % sides
            self.faces.append((self.material, [
                start_ring[index], start_ring[next_index],
                end_ring[next_index], end_ring[index],
            ]))
        self.faces.append((self.material, list(reversed(start_ring))))
        self.faces.append((self.material, end_ring))

    def geometry_summary(self) -> dict:
        minimum = [min(v[i] for v in self.vertices) for i in range(3)]
        maximum = [max(v[i] for v in self.vertices) for i in range(3)]
        return {
            'units': 'centimeters',
            'bounds_cm': {'min': minimum, 'max': maximum},
            'size_cm': [round(maximum[i] - minimum[i], 8) for i in range(3)],
            'vertex_count': len(self.vertices), 'face_count': len(self.faces),
        }

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        mtl_path = self.path.with_suffix('.mtl')
        with mtl_path.open('w', encoding='ascii') as f:
            f.write('# Approximate visual materials; not product color calibration.\n')
            for name, color in self.materials.items():
                f.write(f'newmtl {name}\nKd {color[0]:.5f} {color[1]:.5f} {color[2]:.5f}\nKa 0.08 0.08 0.08\nKs 0.25 0.25 0.25\nNs 32\n\n')
        with self.path.open('w', encoding='ascii') as f:
            f.write(f'mtllib {mtl_path.name}\n')
            f.write('# Units: centimetres; local +X points toward the optical/forward direction; +Z is up.\n')
            for x, y, z in self.vertices:
                f.write(f'v {x:.6f} {y:.6f} {z:.6f}\n')
            active = None
            for material, face in self.faces:
                if material != active:
                    f.write(f'usemtl {material}\n')
                    active = material
                f.write('f ' + ' '.join(str(i) for i in face) + '\n')


def build_camera(output_dir: str | Path = DEFAULT_OUTPUT / 'camera') -> dict:
    output_dir = Path(output_dir)
    out = output_dir / 'approximate_camera_shell.obj'
    obj = ObjWriter(out, {
        'housing': (0.08, 0.09, 0.10),
        'mount': (0.18, 0.20, 0.22),
        'bezel': (0.015, 0.020, 0.025),
        'glass': (0.005, 0.030, 0.070),
    })
    # Local origin is the lens front face: the housing extends behind it.
    obj.use('housing')
    obj.box(center=(-2.35, 0.0, 0.0), size=(2.9, 2.6, 1.8))  # 29 x 26 x 18 mm
    obj.use('mount')
    obj.box(center=(-2.25, 0.0, -1.02), size=(3.1, 2.9, 0.24))
    obj.box(center=(-3.78, 0.0, 0.15), size=(0.45, 1.6, 0.9))  # rear connector block
    obj.use('bezel')
    obj.cylinder_x(-1.05, -0.42, 0.0, 0.0, 0.72)
    obj.cylinder_x(-0.42, -0.06, 0.0, 0.0, 0.58)
    obj.use('glass')
    obj.cylinder_x(-0.06, 0.0, 0.0, 0.0, 0.45)
    obj.use('mount')
    for y in (-1.0, 1.0):
        obj.cylinder_z(-2.6, y, -1.28, -1.02, 0.18, sides=16)
    obj.save()
    manifest = {
        'model': 'KS2A443-3.0 approximate visual shell',
        'geometry': obj.geometry_summary(),
        'approximate': True,
        'reported_reference_parameters_not_calibrated': {'focal_length_mm': 2.8, 'field_of_view_deg': 112, 'nominal_fps': 60},
        'unverified_visual_envelope_cm': {'x_length': 4.005, 'y_width': 2.9, 'z_height': 2.18},
        'local_frame': {'optical_front': '+X at x=0', 'up': '+Z'},
        'note': 'Housing dimensions are a visual approximation only; no manufacturer CAD or dimensional drawing is included.'
    }
    (output_dir / 'approximate_model_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return manifest


def build_lidar_mount(output_dir: str | Path = DEFAULT_OUTPUT / 'lidar_mount') -> dict:
    output_dir = Path(output_dir)
    out = output_dir / 'mid360s_mount_plate.obj'
    obj = ObjWriter(out, {'plate': (0.055, 0.065, 0.075), 'standoff': (0.25, 0.27, 0.29)})
    # Local origin is the shared sensor-deck center. Keep the Livox area 100 x 100 mm;
    # two narrow forward rails and a crossbar support the camera without a broad tray.
    obj.use('plate')
    obj.box(center=(0.0, 0.0, 0.0), size=(10.0, 10.0, 0.30))
    for y in (-1.65, 1.65):
        obj.box(center=(6.85, y, 0.0), size=(3.70, 0.90, 0.30))
    obj.use('standoff')
    for x in (-2.4, 2.4):
        for y in (-1.8, 1.8):
            obj.cylinder_z(x, y, 0.15, 1.52, 0.22, sides=20)  # 4.4 mm post, 13.7 mm high
    obj.save()
    manifest = {
        'model': 'Livox Mid-360S compact central plate with camera rails',
        'geometry': obj.geometry_summary(),
        'approximate': {'standoff_diameter_mm': 4.4, 'standoff_height_mm': 13.7},
        'manual_dimensions': {
            'lidar_platform_mm': [100, 100, 3],
            'plate_and_rails_envelope_mm_without_posts': [137, 100, 3],
            'camera_rails_mm': {'length': 37, 'width_each': 9, 'spacing_center_mm': 33},
            'livox_hole_pitch_mm': [48, 36],
            'holes': '4x M3',
        },
        'local_frame': {'origin': 'plate center', 'up': '+Z'},
        'camera_mount_assumption': {'camera_origin_cm_from_body': [8.5, 0.0, 1.0], 'optical_axis': '+X forward'},
        'note': 'The central 100x100 mm area carries the LiDAR; a narrow open rail bracket supports the forward camera. Geometry is visual only; no thermal or structural simulation is claimed.'
    }
    (output_dir / 'mount_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return manifest


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    camera = build_camera(args.output_dir / 'camera')
    mount = build_lidar_mount(args.output_dir / 'lidar_mount')
    print(json.dumps({'camera': camera['geometry'], 'mount': mount['geometry']}, indent=2))


if __name__ == '__main__':
    main()
