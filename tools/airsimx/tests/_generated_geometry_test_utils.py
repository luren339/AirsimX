"""Readers independent of the generators, shared by geometry tests."""

import json
import struct
from pathlib import Path

import numpy as np


TOOLS = Path(__file__).resolve().parents[1]


def read_glb(path):
    data = Path(path).read_bytes()
    magic, version, total = struct.unpack_from('<4sII', data)
    if (magic, version, total) != (b'glTF', 2, len(data)):
        raise AssertionError('invalid GLB header')
    json_size, json_type = struct.unpack_from('<I4s', data, 12)
    if json_type != b'JSON' or json_size % 4:
        raise AssertionError('invalid JSON chunk')
    document = json.loads(data[20:20 + json_size])
    binary_offset = 20 + json_size
    binary_size, binary_type = struct.unpack_from('<I4s', data, binary_offset)
    if binary_type != b'BIN\0' or binary_offset + 8 + binary_size != total:
        raise AssertionError('invalid BIN chunk')
    return document, data[binary_offset + 8:]


def accessor_values(document, binary, index):
    accessor = document['accessors'][index]
    view = document['bufferViews'][accessor['bufferView']]
    components = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3}[accessor['type']]
    dtype = {5126: '<f4', 5125: '<u4'}[accessor['componentType']]
    offset = view.get('byteOffset', 0) + accessor.get('byteOffset', 0)
    count = accessor['count'] * components
    required = count * 4
    if offset + required > view.get('byteOffset', 0) + view['byteLength']:
        raise AssertionError('accessor exceeds buffer view')
    return np.frombuffer(binary, dtype=dtype, count=count, offset=offset).reshape(-1, components)


def read_obj(path):
    vertices, faces = [], []
    for line in Path(path).read_text(encoding='ascii').splitlines():
        if line.startswith('v '):
            vertices.append([float(value) for value in line.split()[1:]])
        elif line.startswith('f '):
            faces.append([int(value.split('/')[0]) - 1 for value in line.split()[1:]])
    return np.asarray(vertices), faces
