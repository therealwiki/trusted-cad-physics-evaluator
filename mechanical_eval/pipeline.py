from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .materials import IsotropicMaterial, PLA_BASELINE
from .mesh import GmshOccBackend, MeshingBackend, VolumeMesh, initial_overlap_pairs
from .schemas import SubmissionManifest, TaskSpec


@dataclass(frozen=True)
class ComponentProperties:
    name: str
    volume_m3: float
    mass_kg: float
    center_of_mass_m: tuple[float, float, float]
    inertia_kg_m2: tuple[tuple[float, float, float], ...]
    bounds_m: tuple[tuple[float, float, float], tuple[float, float, float]]


@dataclass(frozen=True)
class PreparedSubmission:
    components: tuple[VolumeMesh, ...]
    properties: tuple[ComponentProperties, ...]
    broad_phase_overlap_pairs: tuple[tuple[int, int], ...]
    warnings: tuple[str, ...]


class SubmissionPipeline:
    def __init__(self, backend: MeshingBackend | None = None):
        self.backend = backend or GmshOccBackend()

    def prepare(self, manifest: SubmissionManifest, task: TaskSpec,
                material: IsotropicMaterial = PLA_BASELINE) -> PreparedSubmission:
        ports = {p.name: p for p in manifest.ports}
        missing = set(task.required_ports) - set(ports)
        if missing:
            raise ValueError(f"missing required ports: {sorted(missing)}")
        for name, standard in task.connector_standards.items():
            if ports[name].connector_standard != standard:
                raise ValueError(f"port {name!r} must use task-prescribed standard {standard!r}")
        meshes = self.backend.mesh_step(Path(manifest.step_path), size_m=task.settings.volume_size_m)
        props = []
        for mesh in meshes:
            if mesh.volume_m3 <= 0 or not len(mesh.surface_triangles):
                raise ValueError(f"invalid solid {mesh.name}: no positive closed volume")
            mass, center, inertia = mesh.mass_properties(material.density_kg_m3)
            props.append(ComponentProperties(
                mesh.name, mesh.volume_m3, mass, tuple(center), tuple(map(tuple, inertia)),
                (tuple(mesh.vertices_m.min(0)), tuple(mesh.vertices_m.max(0))),
            ))
        overlaps = initial_overlap_pairs(meshes)
        warnings = (("broad-phase AABB overlap requires exact/contact-phase inspection",) if overlaps else ())
        return PreparedSubmission(tuple(meshes), tuple(props), tuple(overlaps), warnings)
