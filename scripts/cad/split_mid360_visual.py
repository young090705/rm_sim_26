#!/usr/bin/env python3
"""Split MID360 housing from the flat CAD GLB, preserving geometry and poses.

Run again after regenerating gimbal_link.glb. Housing belongs to the lidar link
so RGL ignores the sensor's own shell when tracing rays.
"""
import copy
import json
from pathlib import Path
import struct


def read_glb(path):
    raw = Path(path).read_bytes()
    if struct.unpack_from('<III', raw) != (0x46546C67, 2, len(raw)):
        raise ValueError(f'Invalid GLB: {path}')
    size, kind = struct.unpack_from('<II', raw, 12)
    if kind != 0x4E4F534A:
        raise ValueError('Missing GLB JSON chunk')
    doc = json.loads(raw[20:20 + size])
    binary_size, kind = struct.unpack_from('<II', raw, 20 + size)
    if kind != 0x004E4942:
        raise ValueError('Missing GLB BIN chunk')
    return doc, raw[28 + size:28 + size + binary_size]


def accessor_bytes(doc, binary, index):
    view = doc['bufferViews'][doc['accessors'][index]['bufferView']]
    begin = view.get('byteOffset', 0)
    return binary[begin:begin + view['byteLength']]


def _subset(doc, binary, sensor_part):
    out = copy.deepcopy(doc)
    nodes = [node for node in out['nodes']
             if node['name'].startswith('MID-360(') == sensor_part]
    if not nodes:
        raise ValueError('Empty selection; check CAD MID360 node names')
    meshes = sorted({node['mesh'] for node in nodes})
    out['meshes'] = [out['meshes'][idx] for idx in meshes]
    for node in nodes:
        node['mesh'] = meshes.index(node['mesh'])
    out['nodes'] = nodes
    out['scenes'] = [{'nodes': list(range(len(nodes)))}]
    out['scene'] = 0
    accessors = set()
    for mesh in out['meshes']:
        for primitive in mesh['primitives']:
            accessors.add(primitive['indices'])
            accessors.update(primitive['attributes'].values())
    accessors = sorted(accessors)
    for mesh in out['meshes']:
        for primitive in mesh['primitives']:
            primitive['indices'] = accessors.index(primitive['indices'])
            primitive['attributes'] = {
                key: accessors.index(value)
                for key, value in primitive['attributes'].items()}
    out['accessors'] = [out['accessors'][idx] for idx in accessors]
    views = sorted({accessor['bufferView'] for accessor in out['accessors']})
    for accessor in out['accessors']:
        accessor['bufferView'] = views.index(accessor['bufferView'])
    out['bufferViews'] = [out['bufferViews'][idx] for idx in views]
    packed = bytearray()
    for view in out['bufferViews']:
        packed.extend(b'\0' * (-len(packed) % 4))
        begin = view.get('byteOffset', 0)
        data = binary[begin:begin + view['byteLength']]
        view['byteOffset'] = len(packed)
        packed.extend(data)
    out['buffers'] = [{'byteLength': len(packed)}]
    return out, packed


def _write_glb(path, doc, binary):
    encoded = json.dumps(doc, ensure_ascii=False, separators=(',', ':')).encode()
    encoded += b' ' * (-len(encoded) % 4)
    binary += b'\0' * (-len(binary) % 4)
    Path(path).write_bytes(
        struct.pack('<III', 0x46546C67, 2, 28 + len(encoded) + len(binary))
        + struct.pack('<II', len(encoded), 0x4E4F534A) + encoded
        + struct.pack('<II', len(binary), 0x004E4942) + binary)


def split_mid360_visuals(source, structure, housing):
    doc, binary = read_glb(source)
    # Refuse changed export layouts rather than silently dropping geometry.
    if any(doc.get(key) for key in ('animations', 'images', 'skins')):
        raise ValueError('Expected the flat, untextured CAD export')
    if any('children' in node or 'mesh' not in node for node in doc['nodes']):
        raise ValueError('Expected mesh nodes without a hierarchy')
    if any('sparse' in accessor for accessor in doc['accessors']):
        raise ValueError('Sparse accessors are not supported')
    for path, sensor_part in ((structure, False), (housing, True)):
        _write_glb(path, *_subset(doc, binary, sensor_part))


if __name__ == '__main__':
    meshes = Path(__file__).resolve().parents[2] / 'models/sentry/meshes'
    split_mid360_visuals(
        meshes / 'gimbal_link.glb', meshes / 'gimbal_link_structure.glb',
        meshes / 'mid360_housing.glb')
    print('Generated gimbal_link_structure.glb and mid360_housing.glb')
