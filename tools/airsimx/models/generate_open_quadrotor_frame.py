#!/usr/bin/env python3
"""Generate an approximate open-arm quadrotor frame visual inspired by the user reference."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from generate_sensor_rig_meshes import ObjWriter

DEFAULT_OUTPUT = Path(__file__).resolve().parents[3] / 'build/airsimx/models/frame'


def build_open_quadrotor_frame(output_dir: str | Path = DEFAULT_OUTPUT) -> dict:
    output_dir = Path(output_dir)
    out = output_dir / 'OpenQuadrotorFrame30cm.obj'
    obj = ObjWriter(out, {
        'carbon': (0.075, 0.088, 0.105),
        'motor': (0.20, 0.22, 0.24),
        'electronics': (0.08, 0.09, 0.10),
        'rubber': (0.025, 0.030, 0.035),
        'metal': (0.25, 0.27, 0.29),
        'motor_accent': (0.36, 0.055, 0.04),
    })

    motor_positions = [(-10.0, -10.0), (10.0, -10.0), (-10.0, 10.0), (10.0, 10.0)]

    # Keep the center low and compact, like a practical open-frame inspection quad.
    obj.use('carbon')
    for mx, my in motor_positions:
        sx = 2.7 if mx > 0 else -2.7
        sy = 2.7 if my > 0 else -2.7
        obj.tube_between_xy((sx, sy), (mx, my), width=1.05, height=0.95, z_center=0.05, sides=10)
    # Two thin carbon plates and one compact electronics enclosure replace the
    # earlier three-box stack that visually buried the camera and sensor mounts.
    obj.box(center=(0.0, 0.0, 0.10), size=(6.9, 5.8, 0.28))
    obj.box(center=(0.0, 0.0, 1.18), size=(6.7, 5.6, 0.24))
    obj.use('electronics')
    obj.box(center=(0.0, 0.0, 0.63), size=(5.4, 4.5, 0.72))

    # Motor cans sit immediately below the existing AirSim propeller components.
    obj.use('motor')
    for mx, my in motor_positions:
        obj.cylinder_z(mx, my, -0.78, 0.35, 1.00, sides=28)
        obj.cylinder_z(mx, my, 0.35, 0.72, 0.76, sides=28)
        obj.cylinder_z(mx, my, -0.92, -0.72, 1.10, sides=28)
        obj.use('motor_accent')
        obj.cylinder_z(mx, my, -0.20, -0.05, 1.015, sides=28)
        obj.use('motor')

    # Landing gear: two long low skids and four short supports, matching the reference style.
    obj.use('rubber')
    obj.cylinder_x(-15.0, 15.0, -5.2, -5.05, 0.42, sides=28)
    obj.cylinder_x(-15.0, 15.0, 5.2, -5.05, 0.42, sides=28)
    for x in (-5.7, 5.7):
        for y in (-5.2, 5.2):
            obj.cylinder_z(x, y, -4.85, -0.65, 0.32, sides=20)

    # Four posts support the LiDAR plate. The front camera hangs below the deck
    # on two side struts and short diagonal braces so its lens remains distinct.
    obj.use('metal')
    for x in (-4.15, 4.15):
        for y in (-4.15, 4.15):
            obj.cylinder_z(x, y, 2.35, 3.50, 0.28, sides=20)
    for sign in (-1.0, 1.0):
        y = 1.65 * sign
        obj.cylinder_z(6.55, y, 0.0, 3.80, 0.22, sides=18)
        obj.beam_xy((6.55, y), (8.10, 1.50 * sign), width=0.30, height=0.28, z_center=1.05)

    obj.save()
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = {
        'model': 'open-arm four-rotor UAV visual frame',
        'source': 'procedural approximation following the user-provided open-frame quadrotor reference',
        'approximate': True,
        'geometry': obj.geometry_summary(),
        'target_frame_length_cm': 30.0,
        'visible_frame_bounds_cm': [30.0, 22.20, 9.27],
        'motor_centers_body_cm': [[x, y, 0.0] for x, y in motor_positions],
        'estimated_propeller_tip_envelope_cm': [28.4, 28.4],
        'landing_skids': {'length_cm': 30.0, 'center_y_cm': [-5.2, 5.2], 'center_z_cm': -5.05},
        'sensor_interface': {
            'lidar_platform': '100 x 100 mm central area on separate mount mesh',
            'camera_support': 'two narrow forward rails with side struts to a low front camera',
        },
        'style': 'slim round arms, compact center stack, four exposed motors, front camera bracket, and twin skids',
        'note': 'Illustrative frame geometry, not vendor CAD; AirSim mass, inertia, thrust and collision response are not calibrated by this visual asset.'
    }
    (output_dir / 'open_quadrotor_frame_manifest.json').write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
    )
    return manifest


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    print(json.dumps(build_open_quadrotor_frame(args.output_dir), indent=2))


if __name__ == '__main__':
    main()
