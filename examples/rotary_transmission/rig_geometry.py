from __future__ import annotations

import math

import numpy as np


def d_shaft_mesh(radius_m: float = 0.003, flat_x_m: float = 0.00225,
                 length_m: float = 0.018, segments: int = 48) -> tuple[np.ndarray, np.ndarray]:
    alpha = math.acos(flat_x_m / radius_m)
    angles = np.linspace(alpha, 2 * math.pi - alpha, segments, endpoint=True)
    profile = np.column_stack((radius_m * np.cos(angles), radius_m * np.sin(angles)))
    n = len(profile)
    vertices = np.vstack((np.column_stack((profile, np.full(n, -length_m / 2))),
                          np.column_stack((profile, np.full(n, length_m / 2)))))
    triangles: list[tuple[int, int, int]] = []
    for i in range(n):
        j = (i + 1) % n
        triangles.extend(((i, j, n + j), (i, n + j, n + i)))
    for i in range(1, n - 1):
        triangles.extend(((0, i + 1, i), (n, n + i, n + i + 1)))
    return vertices, np.asarray(triangles, dtype=np.int32)


def sleeve_mesh(inner_radius_m: float = 0.00430, outer_radius_m: float = 0.0058,
                length_m: float = 0.004, segments: int = 48) -> tuple[np.ndarray, np.ndarray]:
    a = np.linspace(0, 2 * math.pi, segments, endpoint=False)
    rings = []
    for z in (-length_m / 2, length_m / 2):
        for r in (inner_radius_m, outer_radius_m):
            rings.append(np.column_stack((r * np.cos(a), r * np.sin(a), np.full(segments, z))))
    vertices = np.vstack(rings)
    triangles: list[tuple[int, int, int]] = []
    for ring_a, ring_b, flip in ((0, 2, False), (1, 3, True), (0, 1, True), (2, 3, False)):
        for i in range(segments):
            j = (i + 1) % segments
            a0, a1 = ring_a * segments + i, ring_a * segments + j
            b0, b1 = ring_b * segments + i, ring_b * segments + j
            triangles.extend(((a0, a1, b1), (a0, b1, b0)) if not flip else ((a0, b1, a1), (a0, b0, b1)))
    return vertices, np.asarray(triangles, dtype=np.int32)


def solid_inertia(vertices: np.ndarray, triangles: np.ndarray, mass: float) -> np.ndarray:
    import trimesh
    mesh = trimesh.Trimesh(vertices, triangles, process=True)
    mesh.density = mass / mesh.volume
    return np.asarray(mesh.moment_inertia)
