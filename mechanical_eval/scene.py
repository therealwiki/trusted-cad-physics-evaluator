from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .materials import IsotropicMaterial, PLA_BASELINE
from .mesh import VolumeMesh
from .schemas import SimulationSettings


@dataclass
class CandidateBody:
    name: str
    point_set: object
    contact: object
    rest_vertices_m: np.ndarray
    tets: np.ndarray


class StarkSceneBuilder:
    """Policy boundary: candidates enter only as deformable contact bodies."""

    def __init__(self, settings: SimulationSettings, work_dir: str | Path = "build/mechanical_eval"):
        import pystark
        work_dir = Path(work_dir).resolve()
        work_dir.mkdir(parents=True, exist_ok=True)
        native = pystark.Settings()
        native.output.simulation_name = "mechanical_eval"
        native.output.output_directory = str(work_dir / "vtk")
        native.output.codegen_directory = str(work_dir / "codegen")
        native.simulation.max_time_step_size = settings.time_step_s
        native.simulation.use_adaptive_time_step = False
        self.simulation = pystark.Simulation(native)
        contact = pystark.EnergyFrictionalContact.GlobalParams()
        contact.default_contact_thickness = settings.contact_distance_m
        contact.friction_enabled = True
        self.simulation.interactions().contact().set_global_params(contact)
        self.settings = settings

    def add_candidate(self, mesh: VolumeMesh, material: IsotropicMaterial = PLA_BASELINE) -> CandidateBody:
        import pystark
        params = pystark.Volume.Params()
        params.inertia.density = material.density_kg_m3
        params.strain.youngs_modulus = material.youngs_modulus_pa
        params.strain.poissons_ratio = material.poissons_ratio
        params.strain.damping = material.damping
        params.contact.contact_thickness = self.settings.contact_distance_m
        handler = self.simulation.presets().deformables().add_volume(
            mesh.name, np.ascontiguousarray(mesh.vertices_m), np.ascontiguousarray(mesh.tets), params
        )
        return CandidateBody(mesh.name, handler.point_set, handler.contact, mesh.vertices_m, mesh.tets)

    def policy_audit(self) -> dict[str, bool]:
        return {"candidate_is_deformable": True, "candidate_attachments": False,
                "candidate_internal_constraints": False, "candidate_prescribed_motion": False}
