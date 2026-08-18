from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pystark

from mechanical_eval.materials import PLA_BASELINE
from mechanical_eval.mesh import VolumeMesh
from mechanical_eval.scene import CandidateBody, StarkSceneBuilder
from mechanical_eval.schemas import SimulationSettings

from rig_geometry import d_shaft_mesh, sleeve_mesh, solid_inertia


@dataclass
class RigScene:
    builder: StarkSceneBuilder
    candidates: list[CandidateBody]
    input_shaft: object
    output_shaft: object
    input_hinge: object
    output_hinge: object


def build(meshes: list[VolumeMesh], work_dir: str, settings: SimulationSettings) -> RigScene:
    builder = StarkSceneBuilder(settings, work_dir)
    sim = builder.simulation
    sim.set_gravity(np.zeros(3))
    candidates = [builder.add_candidate(mesh, PLA_BASELINE) for mesh in meshes]
    candidates.sort(key=lambda body: float(body.rest_vertices_m[:, 0].mean()))
    centers = [(np.asarray(body.rest_vertices_m).min(axis=0) +
                np.asarray(body.rest_vertices_m).max(axis=0)) / 2 for body in candidates]

    contact_params = pystark.EnergyFrictionalContact.Params()
    contact_params.contact_thickness = settings.contact_distance_m / 2
    presets = sim.presets().rigidbodies()
    # Evaluator-owned inertial reference, fixed far outside the candidate envelope.
    ground = presets.add_box("rig_reference", 1.0, np.array([0.002, 0.002, 0.002]), contact_params).handler
    ground.rigidbody.set_translation(np.array([0.0, 0.0, -0.030]))
    sim.rigidbodies().add_constraint_fix(ground.rigidbody)

    shaft_v, shaft_t = d_shaft_mesh()
    physical_shaft_i = solid_inertia(shaft_v, shaft_t, 0.020)
    input_shaft_i = physical_shaft_i.copy()
    output_shaft_i = physical_shaft_i.copy()
    input_shaft_i[2, 2] = 2.0e-4  # Evaluator-owned motor reflected inertia.
    output_shaft_i[2, 2] = 2.0e-5  # Evaluator-owned connector/brake reflected inertia.
    input_shaft = presets.add("input_d_shaft", 0.020, input_shaft_i, shaft_v, shaft_t, contact_params)
    output_shaft = presets.add("output_d_connector", 0.020, output_shaft_i, shaft_v, shaft_t, contact_params)
    input_shaft.rigidbody.set_translation(np.array([centers[0][0], 0.0, 0.0]))
    output_shaft.rigidbody.set_translation(np.array([centers[1][0], 0.0, 0.0]))
    input_hinge = sim.rigidbodies().add_constraint_hinge(
        ground.rigidbody, input_shaft.rigidbody, np.array([centers[0][0], 0.0, 0.0]),
        np.array([0.0, 0.0, 1.0]))
    output_hinge = sim.rigidbodies().add_constraint_hinge(
        ground.rigidbody, output_shaft.rigidbody, np.array([centers[1][0], 0.0, 0.0]),
        np.array([0.0, 0.0, 1.0]))

    sleeve_v, sleeve_t = sleeve_mesh()
    sleeve_i = solid_inertia(sleeve_v, sleeve_t, 0.050)
    sleeves = []
    for body_index, center in enumerate(centers):
        for side, z in (("lower", -0.0061), ("upper", 0.0061)):
            sleeve = presets.add(f"bearing_{body_index}_{side}", 0.050, sleeve_i,
                                  sleeve_v, sleeve_t, contact_params)
            sleeve.rigidbody.set_translation(np.array([center[0], 0.0, z]))
            sim.rigidbodies().add_constraint_fix(sleeve.rigidbody)
            sleeves.append(sleeve)

    for candidate in candidates:
        candidate.contact.set_friction(input_shaft.contact, settings.friction_coefficient)
        candidate.contact.set_friction(output_shaft.contact, settings.friction_coefficient)
        for sleeve in sleeves:
            candidate.contact.set_friction(sleeve.contact, settings.friction_coefficient)
    candidates[0].contact.set_friction(candidates[1].contact, settings.friction_coefficient)
    return RigScene(builder, candidates, input_shaft, output_shaft, input_hinge, output_hinge)
