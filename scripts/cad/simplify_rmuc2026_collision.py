#!/usr/bin/env python3
"""生成 RMUC 2026 结构碰撞网格；原始 STL 继续用于外观和雷达。

依赖 numpy。从仓库根目录执行即可重新生成：
    python3 scripts/cad/simplify_rmuc2026_collision.py

分离完整底板（SDF 中用等尺寸 box 代替），去掉普通地面和 0.2m 平台上
厚度不超过 5mm、距支撑面不超过 6mm 的独立薄片。坡道、台阶等结构保留。
输出保留原始 STL 的单位、坐标及未删除三角形的原始记录。
"""
from pathlib import Path
import struct

import numpy as np


MESH_SCALE = 0.1
WELD_TOLERANCE = 1e-5  # m，仅用于识别相连面，不移动输出顶点
MAX_MARKING_THICKNESS = 0.005
MAX_MARKING_HEIGHT = 0.006


def connected_faces(triangles):
    """按共享顶点识别独立零件，避免只删除薄标线的顶面而留下侧壁。"""
    _, inverse = np.unique(
        np.round(triangles.reshape(-1, 3) / WELD_TOLERANCE).astype(np.int64),
        axis=0, return_inverse=True,
    )
    faces = inverse.reshape(-1, 3)
    parents = np.arange(inverse.max() + 1)
    sizes = np.ones(len(parents), dtype=np.int64)

    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index

    for a, b, c in faces:
        for left, right in ((a, b), (b, c)):
            left, right = find(left), find(right)
            if left != right:
                if sizes[left] < sizes[right]:
                    left, right = right, left
                parents[right] = left
                sizes[left] += sizes[right]
    roots = np.array([find(index) for index in faces[:, 0]])
    return np.unique(roots, return_inverse=True)[1]


def main():
    mesh_dir = Path(__file__).resolve().parents[2] / 'models/rmuc_2026/meshes'
    source = mesh_dir / 'rmuc_2026.stl'
    target = mesh_dir / 'rmuc_2026_collision.stl'
    raw = source.read_bytes()
    count, = struct.unpack_from('<I', raw, 80)
    if len(raw) != 84 + 50 * count:
        raise ValueError('Expected a binary STL with 50-byte triangle records')
    dtype = np.dtype([
        ('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2'),
    ])
    records = np.frombuffer(raw, dtype=dtype, count=count, offset=84)
    triangles = records['vertices'].astype(float) * MESH_SCALE
    labels = connected_faces(triangles)
    component_count = labels.max() + 1
    areas = np.linalg.norm(np.cross(
        triangles[:, 1] - triangles[:, 0],
        triangles[:, 2] - triangles[:, 0],
    ), axis=1) / 2
    floor_label = np.bincount(labels, weights=areas).argmax()
    floor_mask = labels == floor_label
    floor_vertices = triangles[floor_mask].reshape(-1, 3)
    floor_min, floor_max = floor_vertices.min(axis=0), floor_vertices.max(axis=0)
    # 只处理当前场地的完整矩形底板，避免源模型变化后误删其他结构。
    if (floor_mask.sum() != 12
            or not np.allclose(floor_max - floor_min, [29.05, 16.05, 0.2], atol=1e-4)
            or not np.all(np.minimum(abs(floor_vertices - floor_min),
                                     abs(floor_vertices - floor_max)) < 1e-5)):
        raise ValueError('Floor geometry changed; review floor box and filtering first')

    low = np.full(component_count, np.inf)
    high = np.full(component_count, -np.inf)
    np.minimum.at(low, labels, triangles[:, :, 2].min(axis=1))
    np.maximum.at(high, labels, triangles[:, :, 2].max(axis=1))
    markings = np.zeros(component_count, dtype=bool)
    tolerance = 2 * WELD_TOLERANCE
    for level in (floor_max[2], floor_max[2] + 0.2):
        markings |= (
            (low >= level - tolerance)
            & (high <= level + MAX_MARKING_HEIGHT + tolerance)
            & (high - low <= MAX_MARKING_THICKNESS + tolerance)
        )
    keep = ~floor_mask & ~markings[labels]
    header = b'RMUC2026 collision: structural geometry; floor box is defined in SDF'
    target.write_bytes(
        header.ljust(80, b' ') + struct.pack('<I', int(keep.sum()))
        + records[keep].tobytes()
    )
    print(f'Floor removed: {floor_mask.sum()} triangles')
    print(f'Thin markings removed: {markings[labels].sum()} triangles')
    print(f'Collision mesh: {keep.sum()} triangles -> {target}')
    print('Floor box pose in body:', (floor_min + floor_max) / 2)
    print('Floor box size:', floor_max - floor_min)


if __name__ == '__main__':
    main()
