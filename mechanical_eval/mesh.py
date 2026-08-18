from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol

import gmsh
import numpy as np
import trimesh


@dataclass(frozen=True)
class VolumeMesh:
    name: str
    vertices_m: np.ndarray
    tets: np.ndarray
    surface_triangles: np.ndarray

    @property
    def volume_m3(self) -> float:
        x = self.vertices_m[self.tets]
        return float(np.abs(np.linalg.det(x[:, 1:] - x[:, :1])).sum() / 6)

    def mass_properties(self, density_kg_m3: float) -> tuple[float, np.ndarray, np.ndarray]:
        surface = trimesh.Trimesh(self.vertices_m, self.surface_triangles, process=False)
        surface.density = density_kg_m3
        return float(self.volume_m3 * density_kg_m3), np.asarray(surface.center_mass), np.asarray(surface.moment_inertia)


class MeshingBackend(Protocol):
    def mesh_step(self, path: str | Path, *, size_m: float, surface_size_m: float | None = None,
                  source_units: str = "mm") -> list[VolumeMesh]: ...


def tet_surface(tets: np.ndarray) -> np.ndarray:
    faces = np.concatenate((tets[:, [0, 2, 1]], tets[:, [0, 1, 3]], tets[:, [1, 2, 3]], tets[:, [2, 0, 3]]))
    keys = np.sort(faces, axis=1)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    return faces[counts[inverse] == 1].astype(np.int32)


class GmshOccBackend:
    """OpenCASCADE STEP import with one independently indexed tet mesh per volume."""

    def mesh_step(self, path: str | Path, *, size_m: float, surface_size_m: float | None = None,
                  source_units: str = "mm") -> list[VolumeMesh]:
        path = Path(path).resolve()
        if not path.exists():
            raise FileNotFoundError(path)
        scale = {"mm": 1e-3, "m": 1.0}.get(source_units)
        if scale is None:
            raise ValueError("source_units must be 'mm' or 'm'")
        gmsh.initialize()
        try:
            gmsh.option.setNumber("General.Terminal", 0)
            gmsh.model.add(path.stem)
            entities = gmsh.model.occ.importShapes(str(path), highestDimOnly=False)
            gmsh.model.occ.synchronize()
            volumes = sorted(tag for dim, tag in gmsh.model.getEntities(3))
            if not volumes:
                raise ValueError("STEP contains no solid volume entities")
            surface_size_m = size_m if surface_size_m is None else surface_size_m
            gmsh.option.setNumber("Mesh.MeshSizeMin", surface_size_m / scale)
            gmsh.option.setNumber("Mesh.MeshSizeMax", size_m / scale)
            gmsh.option.setNumber("Mesh.MeshSizeFromCurvature", 24)
            gmsh.option.setNumber("Mesh.ElementOrder", 1)
            gmsh.model.mesh.generate(3)
            result = []
            for tag in volumes:
                node_tags, coords, _ = gmsh.model.mesh.getNodes(3, tag, includeBoundary=True)
                tag_to_local = {int(n): i for i, n in enumerate(node_tags)}
                elem_types, _, elem_nodes = gmsh.model.mesh.getElements(3, tag)
                tet_nodes: list[int] = []
                for typ, nodes in zip(elem_types, elem_nodes):
                    if gmsh.model.mesh.getElementProperties(typ)[0].startswith("Tetrahedron 4"):
                        tet_nodes.extend(int(n) for n in nodes)
                if not tet_nodes:
                    raise ValueError(f"volume entity {tag} produced no linear tetrahedra")
                tets = np.fromiter((tag_to_local[n] for n in tet_nodes), dtype=np.int32).reshape(-1, 4)
                vertices = np.asarray(coords, dtype=float).reshape(-1, 3) * scale
                result.append(VolumeMesh(f"volume_{tag}", vertices, tets, tet_surface(tets)))
            return result
        finally:
            gmsh.finalize()


def initial_overlap_pairs(meshes: list[VolumeMesh], tolerance_m: float = 1e-7) -> list[tuple[int, int]]:
    """Conservative broad-phase overlap report; never silently repairs placement."""
    bounds = [(m.vertices_m.min(0), m.vertices_m.max(0)) for m in meshes]
    return [(i, j) for i in range(len(meshes)) for j in range(i + 1, len(meshes))
            if np.all(bounds[i][1] > bounds[j][0] + tolerance_m) and np.all(bounds[j][1] > bounds[i][0] + tolerance_m)]
