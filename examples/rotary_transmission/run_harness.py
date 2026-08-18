from __future__ import annotations

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import numpy as np

from mechanical_eval.contract import contract_sha256, load_contract
from mechanical_eval.materials import IsotropicMaterial
from mechanical_eval.mesh import GmshOccBackend
from mechanical_eval.scene import StarkSceneBuilder
from mechanical_eval.schemas import SimulationSettings


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _evaluator_source_hashes() -> dict[str, str]:
    root = Path(__file__).parents[2]
    files = (
        Path(__file__), Path(__file__).parent / "actuation_scene.py",
        Path(__file__).parent / "insertion_scene.py", Path(__file__).parent / "candidate_manifest.py",
        root / "mechanical_eval" / "scene.py", root / "mechanical_eval" / "scoring.py",
        root / "stark" / "extern" / "symx" / "src" / "solver" / "NewtonsMethod.cpp",
    )
    return {str(path.relative_to(root)): _sha256(path) for path in files}


def _prepare_run_dir(run_dir: Path, contract: dict, step_path: Path) -> None:
    if run_dir.exists() and any(run_dir.iterdir()):
        raise FileExistsError(f"immutable run directory already contains artifacts: {run_dir}")
    run_dir.mkdir(parents=True, exist_ok=True)
    contract_snapshot = run_dir / "contract_snapshot.json"
    shutil.copyfile(Path(__file__).parent / "contracts" / "rotary_transmission_v1.json",
                    contract_snapshot)
    (run_dir / "run_manifest.json").write_text(json.dumps({
        "cad_path": str(step_path), "cad_sha256": _sha256(step_path),
        "contract_sha256": contract_sha256(contract),
        "contract_snapshot": contract_snapshot.name,
        "evaluator_source_sha256": _evaluator_source_hashes(),
    }, indent=2) + "\n")


def _simulation_settings(contract: dict, duration: float) -> SimulationSettings:
    mesh = contract["mesh_and_contact"]
    solver = contract["solver"]
    return SimulationSettings(
        surface_size_m=mesh["surface_size_m"],
        volume_size_m=mesh["volume_size_m"],
        contact_distance_m=mesh["ipc_contact_distance_m"],
        time_step_s=mesh["fixed_time_step_s"],
        duration_s=duration,
        friction_coefficient=mesh["friction_coefficient"],
        ipc_min_contact_stiffness=solver["ipc_min_contact_stiffness"],
        ipc_max_contact_stiffness=solver["ipc_max_contact_stiffness"],
        friction_stick_slide_threshold_m_s=mesh["friction_stick_slide_threshold_m_s"],
        newton_residual_tolerance_abs=solver["newton_residual_tolerance_abs"],
        newton_residual_tolerance_rel=solver["newton_residual_tolerance_rel"],
        newton_step_tolerance=solver["newton_step_tolerance"],
        newton_max_iterations=solver["newton_max_iterations"],
        armijo_backtracking_enabled=solver["armijo_backtracking_enabled"],
        armijo_max_iterations=solver["armijo_max_iterations"],
        invalid_state_max_iterations=solver["invalid_state_max_iterations"],
        hessian_projection_mode=solver["hessian_projection_mode"],
        hessian_projection_epsilon=solver["hessian_projection_epsilon"],
        linear_solver=solver["linear_solver"],
        cg_max_iterations=solver["cg_max_iterations"],
        cg_absolute_tolerance=solver["cg_absolute_tolerance"],
        cg_relative_tolerance=solver["cg_relative_tolerance"],
        cg_stop_on_indefiniteness=solver["cg_stop_on_indefiniteness"],
        cg_bailout_residual=solver["cg_bailout_residual"],
        ilut_fill_factor=solver["ilut_fill_factor"],
        ilut_drop_tolerance=solver["ilut_drop_tolerance"],
        execution_threads=solver["execution_threads"],
    )


def _material(contract: dict) -> IsotropicMaterial:
    material = contract["material"]
    return IsotropicMaterial(
        name=material["profile"], density_kg_m3=material["density_kg_m3"],
        youngs_modulus_pa=material["youngs_modulus_pa"],
        poissons_ratio=material["poissons_ratio"], damping=material["damping"],
        elastic_strain_ceiling=material["elastic_strain_ceiling"],
        invalidity_strain=material["provisional_invalidity_strain"],
        calibration_note=material["print_recipe"],
    )


def _minimum_insertion_duration(contract: dict) -> float:
    return ((contract["rig_geometry"]["initial_insertion_gap_m"] +
             contract["connector"]["engagement_length_m"]) /
            contract["connector"]["insertion_speed_m_s"])


def _rotation_and_omega(points: np.ndarray, velocities: np.ndarray, rest: np.ndarray) -> tuple[float, float, float]:
    center = rest.mean(axis=0)
    r0 = rest[:, :2] - center[:2]
    r = points[:, :2] - points[:, :2].mean(axis=0)
    angle = float(np.arctan2(np.sum(r0[:, 0] * r[:, 1] - r0[:, 1] * r[:, 0]), np.sum(r0 * r)))
    radius = points[:, :2] - points[:, :2].mean(axis=0)
    velocity = velocities[:, :2] - velocities[:, :2].mean(axis=0)
    omega = float(np.sum(radius[:, 0] * velocity[:, 1] - radius[:, 1] * velocity[:, 0]) /
                  max(np.sum(radius * radius), 1e-20))
    wobble = float(np.linalg.norm(points.mean(axis=0)[:2] - center[:2]))
    return angle, omega, wobble


def _deformation_metrics(points: np.ndarray, rest: np.ndarray,
                         tets: np.ndarray) -> tuple[float, float]:
    x, x0 = points[tets], rest[tets]
    ds = np.transpose(x[:, 1:] - x[:, :1], (0, 2, 1))
    dm = np.transpose(x0[:, 1:] - x0[:, :1], (0, 2, 1))
    f = ds @ np.linalg.inv(dm)
    singular = np.linalg.svd(f, compute_uv=False)
    return float(np.max(np.abs(singular - 1.0))), float(np.linalg.det(f).min())


def _max_strain(points: np.ndarray, rest: np.ndarray, tets: np.ndarray) -> float:
    return _deformation_metrics(points, rest, tets)[0]


def _rigid_z_angle(rigidbody: object) -> float:
    rotation = np.asarray(rigidbody.get_rotation_matrix())
    return float(np.arctan2(rotation[1, 0], rotation[0, 0]))


def _advance_with_contact_retries(simulation: object, max_stiffness: float) -> tuple[bool, int]:
    """Advance one physical step, honoring STARK's contact-hardening retry protocol."""
    before = float(simulation.get_time())
    retries = 0
    while True:
        contact = simulation.interactions().contact()
        stiffness_before = float(contact.get_contact_stiffness())
        should_continue = bool(simulation.run_one_time_step())
        if float(simulation.get_time()) > before:
            return True, retries
        stiffness_after = float(contact.get_contact_stiffness())
        if stiffness_after > stiffness_before and stiffness_after <= max_stiffness:
            retries += 1
            continue
        if not should_continue:
            return False, retries
        return False, retries


def run_actuation(step_path: Path, run_dir: Path, duration: float) -> None:
    from actuation_scene import build
    from candidate_manifest import load_and_resolve

    contract_path = Path(__file__).parent / "contracts" / "rotary_transmission_v1.json"
    contract = load_contract(contract_path, allow_prelock=True)
    _prepare_run_dir(run_dir, contract, step_path)
    meshes = GmshOccBackend().mesh_step(step_path, size_m=contract["mesh_and_contact"]["volume_size_m"],
                                       surface_size_m=contract["mesh_and_contact"]["surface_size_m"])
    resolved = load_and_resolve(step_path, meshes)
    settings = _simulation_settings(contract, duration)
    time_step = settings.time_step_s
    rig = build(resolved, str(run_dir), settings, contract, _material(contract))
    frames: list[dict[str, object]] = []
    motor_work = 0.0
    brake_work = 0.0
    previous_time = 0.0
    next_frame = 0.0
    applied_motor_torque = 0.0
    applied_brake_torque = 0.0
    max_motor_torque = 0.0
    max_brake_torque = 0.0
    candidate_raw = []
    candidate_angles = []
    max_wobble = [0.0 for _ in rig.candidates]
    max_escape = [0.0 for _ in rig.candidates]
    max_strain = [0.0 for _ in rig.candidates]
    min_deformation_jacobian = [float("inf") for _ in rig.candidates]
    for body in rig.candidates:
        raw, _, _ = _rotation_and_omega(np.asarray(body.point_set.get_positions()),
                                        np.asarray(body.point_set.get_velocities()), body.rest_vertices_m)
        candidate_raw.append(raw)
        candidate_angles.append(raw)
    shaft_raw = [_rigid_z_angle(rig.input_shaft.rigidbody), _rigid_z_angle(rig.output_shaft.rigidbody)]
    shaft_angles = shaft_raw.copy()
    max_connector_slip = 0.0
    dynamics_samples: list[dict[str, object]] = []

    def sample_integrity() -> list[float]:
        nonlocal max_connector_slip
        omegas = []
        for i, body in enumerate(rig.candidates):
            points = np.asarray(body.point_set.get_positions())
            velocities = np.asarray(body.point_set.get_velocities())
            raw, omega, wobble = _rotation_and_omega(points, velocities, body.rest_vertices_m)
            omegas.append(omega)
            candidate_angles[i] += float(np.arctan2(np.sin(raw - candidate_raw[i]),
                                                    np.cos(raw - candidate_raw[i])))
            candidate_raw[i] = raw
            max_wobble[i] = max(max_wobble[i], wobble)
            max_escape[i] = max(max_escape[i], float(np.linalg.norm(points.mean(0) -
                                                                    body.rest_vertices_m.mean(0))))
            strain, min_j = _deformation_metrics(points, body.rest_vertices_m, body.tets)
            max_strain[i] = max(max_strain[i], strain)
            min_deformation_jacobian[i] = min(min_deformation_jacobian[i], min_j)
        shaft_now = [_rigid_z_angle(rig.input_shaft.rigidbody), _rigid_z_angle(rig.output_shaft.rigidbody)]
        for i, raw in enumerate(shaft_now):
            shaft_angles[i] += float(np.arctan2(np.sin(raw - shaft_raw[i]), np.cos(raw - shaft_raw[i])))
            shaft_raw[i] = raw
        max_connector_slip = max(max_connector_slip,
                                 abs(candidate_angles[0] - shaft_angles[0]),
                                 abs(candidate_angles[1] - shaft_angles[1]))
        return omegas

    sample_integrity()

    def observe() -> None:
        nonlocal motor_work, brake_work, previous_time, next_frame, applied_motor_torque, applied_brake_torque
        nonlocal max_motor_torque, max_brake_torque
        now = float(rig.builder.simulation.get_time())
        dt = max(0.0, now - previous_time)
        previous_time = now
        input_w = float(rig.input_shaft.rigidbody.get_angular_velocity()[2])
        output_w = float(rig.output_shaft.rigidbody.get_angular_velocity()[2])
        actuation = contract["actuation"]
        target_fraction = float(np.clip((now - actuation["speed_ramp_start_s"]) /
                                        actuation["speed_ramp_duration_s"], 0.0, 1.0))
        brake_fraction = float(np.clip((now - actuation["brake_ramp_start_s"]) /
                                       actuation["brake_ramp_duration_s"], 0.0, 1.0))
        target_w = actuation["input_speed_target_rad_s"] * target_fraction
        gain = contract["evaluator_rotor"]["speed_feedback_gain_nm_per_rad_s"]
        limit = actuation["input_torque_limit_nm"]
        applied_motor_torque = float(np.clip(gain * (target_w - input_w), -limit, limit))
        applied_brake_torque = float(-actuation["output_resistance_nm"] * brake_fraction *
                                     np.tanh(output_w / actuation["brake_velocity_regularization_rad_s"]))
        max_motor_torque = max(max_motor_torque, abs(applied_motor_torque))
        max_brake_torque = max(max_brake_torque, abs(applied_brake_torque))
        rig.input_shaft.rigidbody.set_torque(np.array([0.0, 0.0, applied_motor_torque]))
        rig.output_shaft.rigidbody.set_torque(np.array([0.0, 0.0, applied_brake_torque]))
        motor_work += abs(applied_motor_torque * input_w * dt)
        brake_work += abs(applied_brake_torque * output_w * dt)
        if now + 1e-12 < next_frame:
            return
        state = []
        for body in rig.candidates:
            points = np.asarray(body.point_set.get_positions())
            velocities = np.asarray(body.point_set.get_velocities())
            angle, omega, wobble = _rotation_and_omega(points, velocities, body.rest_vertices_m)
            state.append({"angle_rad": angle, "omega_rad_s": omega, "wobble_m": wobble,
                          "points_m": points.tolist()})
        frames.append({"time_s": now, "input_shaft_omega_rad_s": input_w,
                       "output_connector_omega_rad_s": output_w, "motor_torque_nm": applied_motor_torque,
                       "brake_torque_nm": applied_brake_torque,
                       "input_shaft_rotation": np.asarray(rig.input_shaft.rigidbody.get_rotation_matrix()).tolist(),
                       "output_shaft_rotation": np.asarray(rig.output_shaft.rigidbody.get_rotation_matrix()).tolist(),
                       "candidate": state})
        next_frame += 1 / 30

    target_steps = int(np.ceil(duration / time_step))
    completed_steps = 0
    contact_hardening_retries = 0
    for _ in range(target_steps):
        observe()
        advanced, retries = _advance_with_contact_retries(rig.builder.simulation,
                                                          settings.ipc_max_contact_stiffness)
        contact_hardening_retries += retries
        if not advanced:
            break
        completed_steps += 1
        candidate_omegas = sample_integrity()
        dynamics_samples.append({
            "time_s": float(rig.builder.simulation.get_time()),
            "input_gear_omega_rad_s": candidate_omegas[0],
            "output_gear_omega_rad_s": candidate_omegas[1],
            "input_shaft_omega_rad_s": float(rig.input_shaft.rigidbody.get_angular_velocity()[2]),
            "output_connector_omega_rad_s": float(rig.output_shaft.rigidbody.get_angular_velocity()[2]),
            "motor_torque_nm": applied_motor_torque,
            "brake_torque_nm": applied_brake_torque,
        })
    observe()
    final_states = []
    for i, body in enumerate(rig.candidates):
        points = np.asarray(body.point_set.get_positions())
        velocities = np.asarray(body.point_set.get_velocities())
        angle, omega, wobble = _rotation_and_omega(points, velocities, body.rest_vertices_m)
        final_states.append({"angle_rad": candidate_angles[i], "omega_rad_s": omega,
                             "wobble_m": max_wobble[i], "final_wobble_m": wobble,
                             "escape_m": max_escape[i],
                             "final_escape_m": float(np.linalg.norm(points.mean(0) - body.rest_vertices_m.mean(0))),
                             "max_strain": max_strain[i],
                             "min_deformation_jacobian": min_deformation_jacobian[i],
                             "final_strain": _max_strain(points, body.rest_vertices_m, body.tets)})
    np.savez_compressed(run_dir / "recorded_frames.npz", frames=np.asarray(frames, dtype=object))
    steady_dynamics = [row for row in dynamics_samples
                       if float(row["time_s"]) >= max(0.0, duration - 0.2)]
    mean = lambda values: float(np.mean(values)) if values else 0.0
    if len(steady_dynamics) >= 2:
        window_dt = float(steady_dynamics[-1]["time_s"] - steady_dynamics[0]["time_s"])
        output_alpha = ((float(steady_dynamics[-1]["output_connector_omega_rad_s"]) -
                         float(steady_dynamics[0]["output_connector_omega_rad_s"])) /
                        max(window_dt, 1e-12))
        mean_brake_signed = mean([float(row["brake_torque_nm"]) for row in steady_dynamics])
        transmitted_contact_torque = abs(
            contract["evaluator_rotor"]["output_reflected_inertia_kg_m2"] * output_alpha -
            mean_brake_signed)
    else:
        output_alpha = float("nan")
        transmitted_contact_torque = 0.0
    input_shaft_angle, output_shaft_angle = shaft_angles
    observation = {"contract_sha256": contract_sha256(contract), "cad_sha256": _sha256(step_path),
                   "step_path": str(step_path),
                   "duration_s": duration, "candidate": final_states, "input_work_j": motor_work,
                   "output_work_j": brake_work, "applied_motor_torque_nm": applied_motor_torque,
                   "applied_brake_torque_nm": applied_brake_torque,
                   "max_motor_torque_nm": max_motor_torque, "max_brake_torque_nm": max_brake_torque,
                   "completed_steps": completed_steps, "contact_hardening_retries": contact_hardening_retries,
                   "solver_converged": completed_steps == target_steps,
                   "input_shaft_omega_rad_s": float(rig.input_shaft.rigidbody.get_angular_velocity()[2]),
                   "output_connector_omega_rad_s": float(rig.output_shaft.rigidbody.get_angular_velocity()[2]),
                   "input_shaft_angle_rad": input_shaft_angle, "output_connector_angle_rad": output_shaft_angle,
                   "connector_slip_rad": max_connector_slip,
                   "steady_observations": {
                       "input_gear_omega_rad_s": mean([float(f["input_gear_omega_rad_s"])
                                                       for f in steady_dynamics]),
                       "output_gear_omega_rad_s": mean([float(f["output_gear_omega_rad_s"])
                                                        for f in steady_dynamics]),
                       "input_shaft_omega_rad_s": mean([float(f["input_shaft_omega_rad_s"])
                                                        for f in steady_dynamics]),
                       "output_connector_omega_rad_s": mean([float(f["output_connector_omega_rad_s"])
                                                             for f in steady_dynamics]),
                       "motor_torque_nm": mean([float(f["motor_torque_nm"]) for f in steady_dynamics]),
                       "brake_torque_nm": mean([abs(float(f["brake_torque_nm"])) for f in steady_dynamics]),
                       "output_connector_alpha_rad_s2": output_alpha,
                       "transmitted_contact_torque_nm": transmitted_contact_torque},
                   "termination_tuple": {
                       "simulation_time_s": float(rig.builder.simulation.get_time()),
                       "time_since_brake_start_s": max(0.0, float(rig.builder.simulation.get_time()) - 0.5),
                       "instantaneous_brake_torque_nm": abs(applied_brake_torque),
                       "instantaneous_motor_torque_nm": applied_motor_torque,
                       "candidate_angles_rad": [state["angle_rad"] for state in final_states],
                       "candidate_omegas_rad_s": [state["omega_rad_s"] for state in final_states],
                       "max_strain": max(state["max_strain"] for state in final_states),
                       "min_deformation_jacobian": min(state["min_deformation_jacobian"]
                                                        for state in final_states),
                       "max_escape_m": max(state["escape_m"] for state in final_states),
                       "connector_slip_rad": max_connector_slip,
                       "solver_status": "CONVERGED" if completed_steps == target_steps else "INCONCLUSIVE_NUMERICS"},
                   "policy_audit": rig.builder.policy_audit(), "classification": "UNSCORED_PHYSICS_OBSERVATION"}
    (run_dir / "actuation_observation.json").write_text(json.dumps(observation, indent=2) + "\n")


def run_insertion(step_path: Path, run_dir: Path, duration: float) -> None:
    from insertion_scene import build
    from candidate_manifest import load_and_resolve

    contract_path = Path(__file__).parent / "contracts" / "rotary_transmission_v1.json"
    contract = load_contract(contract_path, allow_prelock=True)
    minimum_duration = _minimum_insertion_duration(contract)
    if duration + 1e-12 < minimum_duration:
        raise ValueError(f"insertion duration {duration:g}s cannot reach the required engagement; "
                         f"minimum is {minimum_duration:g}s")
    _prepare_run_dir(run_dir, contract, step_path)
    meshes = GmshOccBackend().mesh_step(step_path, size_m=contract["mesh_and_contact"]["volume_size_m"],
                                       surface_size_m=contract["mesh_and_contact"]["surface_size_m"])
    resolved = load_and_resolve(step_path, meshes)
    settings = _simulation_settings(contract, duration)
    time_step = settings.time_step_s
    rig = build(resolved, str(run_dir), settings, contract, _material(contract))
    force_history: list[dict[str, float]] = []
    target_steps = int(np.ceil(duration / time_step))
    completed_steps = 0
    contact_hardening_retries = 0
    max_force = 0.0
    for _ in range(target_steps):
        advanced, retries = _advance_with_contact_retries(rig.builder.simulation,
                                                          settings.ipc_max_contact_stiffness)
        contact_hardening_retries += retries
        after = float(rig.builder.simulation.get_time())
        if not advanced:
            break
        completed_steps += 1
        in_v, in_f = rig.input_press.get_linear_velocity_constraint().get_signed_velocity_violation_and_force()
        out_v, out_f = rig.output_press.get_linear_velocity_constraint().get_signed_velocity_violation_and_force()
        max_force = max(max_force, abs(float(in_f)), abs(float(out_f)))
        if completed_steps % 10 == 0:
            force_history.append({"time_s": after, "input_force_n": float(in_f), "output_force_n": float(out_f),
                                  "input_velocity_violation_m_s": float(in_v),
                                  "output_velocity_violation_m_s": float(out_v)})
    shaft_length = contract["rig_geometry"]["shaft_length_m"]
    shaft_centers = [float(rig.input_shaft.rigidbody.get_translation()[2]),
                     float(rig.output_shaft.rigidbody.get_translation()[2])]
    engagements = []
    lateral_misalignments = []
    for body, center, shaft in zip(rig.candidates[:2], shaft_centers,
                                   (rig.input_shaft, rig.output_shaft)):
        current = np.asarray(body.point_set.get_positions())
        body_min, body_max = float(current[:, 2].min()), float(current[:, 2].max())
        engagements.append(max(0.0, min(center + shaft_length / 2, body_max) -
                               max(center - shaft_length / 2, body_min)))
        lateral_misalignments.append(float(np.linalg.norm(
            current.mean(axis=0)[:2] - np.asarray(shaft.rigidbody.get_translation())[:2])))
    solver_converged = completed_steps == target_steps
    insertion_passed = (max_force < contract["connector"]["max_insertion_force_n"] and
                        min(engagements) >= contract["connector"]["engagement_length_m"] and
                        max(lateral_misalignments) <= contract["connector"]["diametral_clearance_m"] / 2)
    classification = "INCONCLUSIVE_NUMERICS" if not solver_converged else "PASS" if insertion_passed else "FAIL"
    observation = {
        "contract_sha256": contract_sha256(contract), "cad_sha256": _sha256(step_path),
        "step_path": str(step_path), "duration_s": duration,
        "completed_steps": completed_steps, "contact_hardening_retries": contact_hardening_retries,
        "solver_converged": solver_converged,
        "max_insertion_force_n": max_force, "force_limit_n": contract["connector"]["max_insertion_force_n"],
        "required_engagement_m": contract["connector"]["engagement_length_m"],
        "measured_engagement_m": engagements,
        "connector_lateral_misalignment_m": lateral_misalignments,
        "input_final_translation_m": np.asarray(rig.input_shaft.rigidbody.get_translation()).tolist(),
        "output_final_translation_m": np.asarray(rig.output_shaft.rigidbody.get_translation()).tolist(),
        "force_history": force_history, "policy_audit": rig.builder.policy_audit(),
        "classification": classification
    }
    (run_dir / "insertion_observation.json").write_text(json.dumps(observation, indent=2) + "\n")


def run(step_path: Path, run_dir: Path, duration: float) -> None:
    contract_path = Path(__file__).parent / "contracts" / "rotary_transmission_v1.json"
    contract = load_contract(contract_path, allow_prelock=True)
    _prepare_run_dir(run_dir, contract, step_path)
    meshes = GmshOccBackend().mesh_step(step_path, size_m=contract["mesh_and_contact"]["volume_size_m"],
                                       surface_size_m=contract["mesh_and_contact"]["surface_size_m"])
    for mesh in meshes:
        np.savez_compressed(run_dir / f"{mesh.name}.npz", vertices_m=mesh.vertices_m, tets=mesh.tets,
                            surface_triangles=mesh.surface_triangles)
    builder = StarkSceneBuilder(work_dir=run_dir, settings=_simulation_settings(contract, duration))
    builder.simulation.set_gravity(np.zeros(3))
    bodies = [builder.add_candidate(mesh, _material(contract)) for mesh in meshes]
    initial = [np.asarray(body.point_set.get_positions()).copy() for body in bodies]
    builder.simulation.run(duration)
    final = [np.asarray(body.point_set.get_positions()).copy() for body in bodies]
    observation = {
        "contract_sha256": contract_sha256(contract),
        "cad_sha256": _sha256(step_path),
        "step_path": str(step_path),
        "duration_s": duration,
        "component_count": len(bodies),
        "component_max_displacement_m": [float(np.linalg.norm(b - a, axis=1).max()) for a, b in zip(initial, final)],
        "policy_audit": builder.policy_audit(),
        "classification": "INCONCLUSIVE_NUMERICS",
        "reason": "bring-up run only; no rig actuation or physical reward",
    }
    (run_dir / "bringup_observation.json").write_text(json.dumps(observation, indent=2) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=Path)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--actuation", action="store_true")
    parser.add_argument("--insertion", action="store_true")
    args = parser.parse_args()
    if args.actuation and args.insertion:
        parser.error("choose only one of --actuation or --insertion")
    if args.duration is None:
        cli_contract = load_contract(Path(__file__).parent / "contracts" / "rotary_transmission_v1.json",
                                     allow_prelock=True)
        args.duration = (_minimum_insertion_duration(cli_contract) if args.insertion else
                         cli_contract["actuation"]["actuation_duration_s"] if args.actuation else 0.001)
    (run_insertion if args.insertion else run_actuation if args.actuation else run)(
        args.step.resolve(), args.run_dir.resolve(), args.duration)
