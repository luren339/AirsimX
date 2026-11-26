#!/usr/bin/env python3
"""Convert explicitly supplied Datasmith transforms to the existing C++ schema.

This writes placement data only. It does not provide or import the external
manufacturer/CAD meshes and does not modify the AirSim plugin automatically.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re

DEFAULT_OUTPUT = Path(__file__).resolve().parents[3] / 'build/airsimx/models/UAVSensorRigMeshData.h'


def _vector(values, count, label):
    if len(values) != count or not all(isinstance(x, (int, float)) and math.isfinite(x) for x in values):
        raise ValueError(f'{label} must contain {count} finite numbers')
    return values


def generate_mesh_data(source: str | Path, output: str | Path = DEFAULT_OUTPUT,
                       expected_count: int = 13) -> int:
    data = json.loads(Path(source).read_text(encoding='utf-8'))
    parts = data.get('mesh_actor_instances', [])
    if expected_count <= 0 or len(parts) != expected_count:
        raise ValueError(f'expected {expected_count} Datasmith component instances, got {len(parts)}')
    lines = [
        '// Generated from explicitly supplied Datasmith component transforms.',
        '// External meshes are not included; check their source and redistribution rights.',
        '#pragma once', '#include "CoreMinimal.h"', 'namespace UAVSensorRigMeshData {',
        'struct FPart { const TCHAR* Name; const TCHAR* AssetPath; FVector LocationCm; FQuat RotationXyzw; FVector Scale; };',
        'static const FPart Mid360SParts[] = {',
    ]
    for index, part in enumerate(parts):
        name = f'Mid360SPart_{index:02d}'
        asset = part['static_mesh_asset']
        if not isinstance(asset, str) or not re.fullmatch(r'/[A-Za-z0-9_/]+(?:\.[A-Za-z0-9_]+)?', asset):
            raise ValueError('static_mesh_asset must be an Unreal asset path without quotes or escapes')
        position = _vector(part['translation'], 3, 'translation')
        rotation = _vector(part['rotation_xyzw'], 4, 'rotation_xyzw')
        scale = _vector(part['scale'], 3, 'scale')
        if abs(sum(x * x for x in rotation) - 1.0) > 1e-4 or any(x == 0 for x in scale):
            raise ValueError('rotation must be a unit quaternion and scale must be nonzero')
        lines.append(
            '    {TEXT("%s"), TEXT("%s"), FVector(%.9ff, %.9ff, %.9ff), '
            'FQuat(%.12ff, %.12ff, %.12ff, %.12ff), FVector(%.9ff, %.9ff, %.9ff)},'
            % (name, asset, *position, *rotation, *scale))
    lines += ['};', 'static constexpr int32 Mid360SPartCount = UE_ARRAY_COUNT(Mid360SParts);',
              '} // namespace UAVSensorRigMeshData', '']
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(lines), encoding='utf-8')
    return len(parts)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='external component transform JSON; no default asset source')
    parser.add_argument('--output', type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument('--expected-count', type=int, default=13)
    args = parser.parse_args(argv)
    count = generate_mesh_data(args.source, args.output, args.expected_count)
    print(f'Wrote {args.output} with {count} mesh instances')


if __name__ == '__main__':
    main()
