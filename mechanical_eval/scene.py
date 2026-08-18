from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
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

    def __init__(self, settings: SimulationSettings, work_dir: str | Path = "build/mechanical_eval",
                 codegen_dir: str | Path | None = None):
        import pystark
        work_dir = Path(work_dir).resolve()
        work_dir.mkdir(parents=True, exist_ok=True)
        native = pystark.Settings()
        native.output.simulation_name = "mechanical_eval"
        native.output.output_directory = str(work_dir / "vtk")
        codegen_dir = work_dir / "codegen" if codegen_dir is None else Path(codegen_dir).resolve()
        codegen_dir.mkdir(parents=True, exist_ok=True)
        native.output.codegen_directory = str(codegen_dir)
        native.simulation.max_time_step_size = settings.time_step_s
        native.simulation.use_adaptive_time_step = False
        native.newton.residual_tolerance_abs = settings.newton_residual_tolerance_abs
        native.newton.residual_tolerance_rel = settings.newton_residual_tolerance_rel
        native.newton.step_tolerance = settings.newton_step_tolerance
        native.newton.max_iterations = settings.newton_max_iterations
        native.newton.enable_armijo_backtracking = settings.armijo_backtracking_enabled
        native.newton.max_backtracking_armijo_iterations = settings.armijo_max_iterations
        native.newton.max_backtracking_invalid_state_iterations = settings.invalid_state_max_iterations
        native.newton.projection_mode = pystark.ProjectionToPD.Progressive
        native.newton.projection_eps = settings.hessian_projection_epsilon
        native.newton.linear_solver = {
            "BDPCG": pystark.LinearSolver.BDPCG,
            "EigenICPCG": pystark.LinearSolver.EigenICPCG,
            "EigenILUTBiCGSTAB": pystark.LinearSolver.EigenILUTBiCGSTAB,
        }[settings.linear_solver]
        native.newton.cg_max_iterations = settings.cg_max_iterations
        native.newton.cg_abs_tolerance = settings.cg_absolute_tolerance
        native.newton.cg_rel_tolerance = settings.cg_relative_tolerance
        native.newton.cg_stop_on_indefiniteness = settings.cg_stop_on_indefiniteness
        native.newton.bailout_residual = settings.cg_bailout_residual
        native.newton.ilut_fill_factor = settings.ilut_fill_factor
        native.newton.ilut_drop_tolerance = settings.ilut_drop_tolerance
        if settings.execution_threads:
            native.execution.n_threads = settings.execution_threads
        self.simulation = pystark.Simulation(native)
        contact = pystark.EnergyFrictionalContact.GlobalParams()
        # STARK adds the two objects' thicknesses to obtain the pair barrier
        # distance. The evaluator setting is deliberately the pair distance.
        contact.default_contact_thickness = settings.contact_distance_m / 2
        contact.min_contact_stiffness = settings.ipc_min_contact_stiffness
        contact.max_contact_stiffness = settings.ipc_max_contact_stiffness
        contact.friction_stick_slide_threshold = settings.friction_stick_slide_threshold_m_s
        contact.friction_enabled = True
        self.simulation.interactions().contact().set_global_params(contact)
        self.settings = settings
        self._candidate_construction_ledger: list[dict[str, object]] = []

    def add_candidate(self, mesh: VolumeMesh, material: IsotropicMaterial = PLA_BASELINE) -> CandidateBody:
        import pystark
        params = pystark.Volume.Params()
        params.inertia.density = material.density_kg_m3
        params.strain.youngs_modulus = material.youngs_modulus_pa
        params.strain.poissons_ratio = material.poissons_ratio
        params.strain.damping = material.damping
        params.strain.elasticity_only = True
        params.contact.contact_thickness = self.settings.contact_distance_m / 2
        handler = self.simulation.presets().deformables().add_volume(
            mesh.name, np.ascontiguousarray(mesh.vertices_m), np.ascontiguousarray(mesh.tets), params
        )
        self._candidate_construction_ledger.append({
            "name": mesh.name,
            "body_model": "tetrahedral_fem_deformable",
            "attachment_operations": 0,
            "constraint_operations": 0,
            "prescribed_motion_operations": 0,
            "direct_force_operations": 0,
        })
        return CandidateBody(mesh.name, handler.point_set, handler.contact, mesh.vertices_m, mesh.tets)

    def policy_audit(self) -> dict[str, object]:
        canonical = json.dumps(self._candidate_construction_ledger, sort_keys=True, separators=(",", ":"))
        return {
            "audit_method": "candidate_construction_ledger_v1",
            "candidate_count": len(self._candidate_construction_ledger),
            "candidate_is_deformable": bool(self._candidate_construction_ledger),
            "candidate_attachments": any(row["attachment_operations"] for row in self._candidate_construction_ledger),
            "candidate_internal_constraints": any(row["constraint_operations"] for row in self._candidate_construction_ledger),
            "candidate_prescribed_motion": any(row["prescribed_motion_operations"] for row in self._candidate_construction_ledger),
            "candidate_direct_forces": any(row["direct_force_operations"] for row in self._candidate_construction_ledger),
            "ledger_sha256": hashlib.sha256(canonical.encode()).hexdigest(),
        }
