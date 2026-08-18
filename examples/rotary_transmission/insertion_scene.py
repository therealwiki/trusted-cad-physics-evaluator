from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pystark

from mechanical_eval.materials import IsotropicMaterial
from mechanical_eval.mesh import VolumeMesh
from mechanical_eval.scene import CandidateBody, StarkSceneBuilder
from mechanical_eval.schemas import SimulationSettings

from candidate_manifest import ResolvedCandidate
from rig_geometry import cylinder_mesh, d_shaft_mesh, solid_inertia


@dataclass
class InsertionScene:
    builder: StarkSceneBuilder
    candidates: list[CandidateBody]
    input_shaft: object
    output_shaft: object
    input_press: object
    output_press: object


def build(resolved: ResolvedCandidate, work_dir: str, settings: SimulationSettings,
          contract: dict, material: IsotropicMaterial) -> InsertionScene:
    cache = Path(work_dir).parent.parent / "build" / "codegen" / "shared_scene_v1"
    builder = StarkSceneBuilder(settings, work_dir, codegen_dir=cache)
    sim = builder.simulation
    sim.set_gravity(np.zeros(3))
    roles = ("input_gear", "output_gear", "lower_bearing_plate", "upper_bearing_plate")
    candidates = [builder.add_candidate(resolved.by_role[role], material) for role in roles]
    centers = [resolved.ports_m["input"], resolved.ports_m["output"]]

    contact_params = pystark.EnergyFrictionalContact.Params()
    contact_params.contact_thickness = settings.contact_distance_m / 2
    presets = sim.presets().rigidbodies()
    ground = presets.add_box("rig_reference", 1.0, np.array([0.002, 0.002, 0.002]), contact_params).handler
    ground.rigidbody.set_translation(np.array([0.0, 0.0, -0.030]))
    sim.rigidbodies().add_constraint_fix(ground.rigidbody)

    geometry = contract["rig_geometry"]
    radius = contract["connector"]["nominal_diameter_m"] / 2
    shaft_v, shaft_t = d_shaft_mesh(radius_m=radius,
                                    flat_x_m=radius - contract["connector"]["flat_depth_m"],
                                    length_m=geometry["shaft_length_m"])
    shaft_i = solid_inertia(shaft_v, shaft_t, geometry["shaft_mass_kg"])
    input_shaft = presets.add("input_d_shaft", geometry["shaft_mass_kg"], shaft_i,
                              shaft_v, shaft_t, contact_params)
    output_shaft = presets.add("output_d_connector", geometry["shaft_mass_kg"], shaft_i,
                               shaft_v, shaft_t, contact_params)
    start_offset = geometry["shaft_length_m"] / 2 + max(-body.rest_vertices_m[:, 2].min()
                                                         for body in candidates[:2]) + geometry["initial_insertion_gap_m"]
    input_shaft.rigidbody.set_translation(np.array([centers[0][0], 0.0, -start_offset]))
    output_shaft.rigidbody.set_translation(np.array([centers[1][0], 0.0, start_offset]))
    input_press = sim.rigidbodies().add_constraint_prismatic_press(
        ground.rigidbody, input_shaft.rigidbody, np.array([centers[0][0], 0.0, -start_offset]),
        np.array([0.0, 0.0, 1.0]), contract["connector"]["insertion_speed_m_s"],
        contract["connector"]["max_insertion_force_n"])
    output_press = sim.rigidbodies().add_constraint_prismatic_press(
        ground.rigidbody, output_shaft.rigidbody, np.array([centers[1][0], 0.0, start_offset]),
        np.array([0.0, 0.0, -1.0]), contract["connector"]["insertion_speed_m_s"],
        contract["connector"]["max_insertion_force_n"])

    pin_length = contract["rig_geometry"]["mount_pin_length_m"]
    pin_v, pin_t = cylinder_mesh(contract["mount"]["bolt_diameter_m"] / 2, pin_length)
    pin_i = solid_inertia(pin_v, pin_t, 0.02)
    fixture_pins = []
    for index, (x, y) in enumerate(contract["mount"]["bolt_pattern_m"]):
        for side, z in zip(("lower", "upper"), geometry["bearing_axial_centers_m"]):
            pin = presets.add(f"mount_pin_{index}_{side}", 0.02, pin_i, pin_v, pin_t, contact_params)
            pin.rigidbody.set_translation(np.array([x, y, z]))
            sim.rigidbodies().add_constraint_fix(pin.rigidbody)
            fixture_pins.append(pin)
    for candidate in candidates:
        candidate.contact.set_friction(input_shaft.contact, settings.friction_coefficient)
        candidate.contact.set_friction(output_shaft.contact, settings.friction_coefficient)
        for pin in fixture_pins:
            candidate.contact.set_friction(pin.contact, settings.friction_coefficient)
    for i, candidate in enumerate(candidates):
        for other in candidates[i + 1:]:
            candidate.contact.set_friction(other.contact, settings.friction_coefficient)
    return InsertionScene(builder, candidates, input_shaft, output_shaft, input_press, output_press)
