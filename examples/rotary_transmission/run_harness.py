from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from mechanical_eval.contract import contract_sha256, load_contract
from mechanical_eval.materials import PLA_BASELINE
from mechanical_eval.mesh import GmshOccBackend
from mechanical_eval.scene import StarkSceneBuilder
from mechanical_eval.schemas import SimulationSettings


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
    )


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


def _max_strain(points: np.ndarray, rest: np.ndarray, tets: np.ndarray) -> float:
    x, x0 = points[tets], rest[tets]
    ds = np.transpose(x[:, 1:] - x[:, :1], (0, 2, 1))
    dm = np.transpose(x0[:, 1:] - x0[:, :1], (0, 2, 1))
    f = ds @ np.linalg.inv(dm)
    singular = np.linalg.svd(f, compute_uv=False)
    return float(np.max(np.abs(singular - 1.0)))


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
        simulation.run_one_time_step()
        if float(simulation.get_time()) > before:
            return True, retries
        stiffness_after = float(contact.get_contact_stiffness())
        if stiffness_after > stiffness_before and stiffness_after <= max_stiffness:
            retries += 1
            continue
        return False, retries


def _wrapped_angle_distance(a: float, b: float) -> float:
    return abs(float(np.arctan2(np.sin(a - b), np.cos(a - b))))


def run_actuation(step_path: Path, run_dir: Path, duration: float) -> None:
    from actuation_scene import build

    contract_path = Path(__file__).parent / "contracts" / "rotary_transmission_v1.json"
    contract = load_contract(contract_path, allow_prelock=True)
    meshes = GmshOccBackend().mesh_step(step_path, size_m=contract["mesh_and_contact"]["volume_size_m"],
                                       surface_size_m=contract["mesh_and_contact"]["surface_size_m"])
    run_dir.mkdir(parents=True, exist_ok=True)
    settings = _simulation_settings(contract, duration)
    time_step = settings.time_step_s
    rig = build(meshes, str(run_dir), settings)
    frames: list[dict[str, object]] = []
    motor_work = 0.0
    brake_work = 0.0
    previous_time = 0.0
    next_frame = 0.0
    applied_motor_torque = 0.0
    applied_brake_torque = 0.0

    def observe() -> None:
        nonlocal motor_work, brake_work, previous_time, next_frame, applied_motor_torque, applied_brake_torque
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
                                     np.tanh(output_w / 0.10))
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
    observe()
    final_states = []
    for body in rig.candidates:
        points = np.asarray(body.point_set.get_positions())
        velocities = np.asarray(body.point_set.get_velocities())
        angle, omega, wobble = _rotation_and_omega(points, velocities, body.rest_vertices_m)
        final_states.append({"angle_rad": angle, "omega_rad_s": omega, "wobble_m": wobble,
                             "escape_m": float(np.linalg.norm(points.mean(0) - body.rest_vertices_m.mean(0))),
                             "max_strain": _max_strain(points, body.rest_vertices_m, body.tets)})
    np.savez_compressed(run_dir / "recorded_frames.npz", frames=np.asarray(frames, dtype=object))
    steady = [frame for frame in frames if float(frame["time_s"]) >= max(0.0, duration - 0.2)]
    mean = lambda values: float(np.mean(values)) if values else 0.0
    input_shaft_angle = _rigid_z_angle(rig.input_shaft.rigidbody)
    output_shaft_angle = _rigid_z_angle(rig.output_shaft.rigidbody)
    observation = {"contract_sha256": contract_sha256(contract), "step_path": str(step_path),
                   "duration_s": duration, "candidate": final_states, "input_work_j": motor_work,
                   "output_work_j": brake_work, "applied_motor_torque_nm": applied_motor_torque,
                   "applied_brake_torque_nm": applied_brake_torque,
                   "completed_steps": completed_steps, "contact_hardening_retries": contact_hardening_retries,
                   "solver_converged": completed_steps == target_steps,
                   "input_shaft_omega_rad_s": float(rig.input_shaft.rigidbody.get_angular_velocity()[2]),
                   "output_connector_omega_rad_s": float(rig.output_shaft.rigidbody.get_angular_velocity()[2]),
                   "input_shaft_angle_rad": input_shaft_angle, "output_connector_angle_rad": output_shaft_angle,
                   "connector_slip_rad": max(_wrapped_angle_distance(final_states[0]["angle_rad"], input_shaft_angle),
                                             _wrapped_angle_distance(final_states[1]["angle_rad"], output_shaft_angle)),
                   "steady_observations": {
                       "input_gear_omega_rad_s": mean([f["candidate"][0]["omega_rad_s"] for f in steady]),
                       "output_gear_omega_rad_s": mean([f["candidate"][1]["omega_rad_s"] for f in steady]),
                       "input_shaft_omega_rad_s": mean([f["input_shaft_omega_rad_s"] for f in steady]),
                       "output_connector_omega_rad_s": mean([f["output_connector_omega_rad_s"] for f in steady]),
                       "motor_torque_nm": mean([f["motor_torque_nm"] for f in steady]),
                       "brake_torque_nm": mean([abs(f["brake_torque_nm"]) for f in steady])},
                   "termination_tuple": {
                       "simulation_time_s": float(rig.builder.simulation.get_time()),
                       "time_since_brake_start_s": max(0.0, float(rig.builder.simulation.get_time()) - 0.5),
                       "instantaneous_brake_torque_nm": abs(applied_brake_torque),
                       "instantaneous_motor_torque_nm": applied_motor_torque,
                       "candidate_angles_rad": [state["angle_rad"] for state in final_states],
                       "candidate_omegas_rad_s": [state["omega_rad_s"] for state in final_states],
                       "max_strain": max(state["max_strain"] for state in final_states),
                       "max_escape_m": max(state["escape_m"] for state in final_states),
                       "connector_slip_rad": max(_wrapped_angle_distance(final_states[0]["angle_rad"], input_shaft_angle),
                                                 _wrapped_angle_distance(final_states[1]["angle_rad"], output_shaft_angle)),
                       "solver_status": "CONVERGED" if completed_steps == target_steps else "INCONCLUSIVE_NUMERICS"},
                   "policy_audit": rig.builder.policy_audit(), "classification": "UNSCORED_PHYSICS_OBSERVATION"}
    (run_dir / "actuation_observation.json").write_text(json.dumps(observation, indent=2) + "\n")


def run_insertion(step_path: Path, run_dir: Path, duration: float) -> None:
    from insertion_scene import build

    contract_path = Path(__file__).parent / "contracts" / "rotary_transmission_v1.json"
    contract = load_contract(contract_path, allow_prelock=True)
    meshes = GmshOccBackend().mesh_step(step_path, size_m=contract["mesh_and_contact"]["volume_size_m"],
                                       surface_size_m=contract["mesh_and_contact"]["surface_size_m"])
    run_dir.mkdir(parents=True, exist_ok=True)
    settings = _simulation_settings(contract, duration)
    time_step = settings.time_step_s
    rig = build(meshes, str(run_dir), settings)
    force_history: list[dict[str, float]] = []
    target_steps = int(np.ceil(duration / time_step))
    completed_steps = 0
    contact_hardening_retries = 0
    for _ in range(target_steps):
        advanced, retries = _advance_with_contact_retries(rig.builder.simulation,
                                                          settings.ipc_max_contact_stiffness)
        contact_hardening_retries += retries
        after = float(rig.builder.simulation.get_time())
        if not advanced:
            break
        completed_steps += 1
        if completed_steps % 10 == 0:
            in_v, in_f = rig.input_press.get_linear_velocity_constraint().get_signed_velocity_violation_and_force()
            out_v, out_f = rig.output_press.get_linear_velocity_constraint().get_signed_velocity_violation_and_force()
            force_history.append({"time_s": after, "input_force_n": float(in_f), "output_force_n": float(out_f),
                                  "input_velocity_violation_m_s": float(in_v),
                                  "output_velocity_violation_m_s": float(out_v)})
    max_force = max((max(abs(row["input_force_n"]), abs(row["output_force_n"])) for row in force_history), default=0.0)
    observation = {
        "contract_sha256": contract_sha256(contract), "step_path": str(step_path), "duration_s": duration,
        "completed_steps": completed_steps, "contact_hardening_retries": contact_hardening_retries,
        "solver_converged": completed_steps == target_steps,
        "max_insertion_force_n": max_force, "force_limit_n": contract["connector"]["max_insertion_force_n"],
        "input_final_translation_m": np.asarray(rig.input_shaft.rigidbody.get_translation()).tolist(),
        "output_final_translation_m": np.asarray(rig.output_shaft.rigidbody.get_translation()).tolist(),
        "force_history": force_history, "policy_audit": rig.builder.policy_audit(),
        "classification": "PASS" if completed_steps == target_steps and max_force < contract["connector"]["max_insertion_force_n"] else "INCONCLUSIVE_NUMERICS"
    }
    (run_dir / "insertion_observation.json").write_text(json.dumps(observation, indent=2) + "\n")


def run(step_path: Path, run_dir: Path, duration: float) -> None:
    contract_path = Path(__file__).parent / "contracts" / "rotary_transmission_v1.json"
    contract = load_contract(contract_path, allow_prelock=True)
    meshes = GmshOccBackend().mesh_step(step_path, size_m=contract["mesh_and_contact"]["volume_size_m"],
                                       surface_size_m=contract["mesh_and_contact"]["surface_size_m"])
    run_dir.mkdir(parents=True, exist_ok=True)
    for mesh in meshes:
        np.savez_compressed(run_dir / f"{mesh.name}.npz", vertices_m=mesh.vertices_m, tets=mesh.tets,
                            surface_triangles=mesh.surface_triangles)
    builder = StarkSceneBuilder(work_dir=run_dir, settings=__import__("mechanical_eval.tasks", fromlist=["RotaryTransmissionSpec"]).RotaryTransmissionSpec().task_spec().settings)
    builder.simulation.set_gravity(np.zeros(3))
    bodies = [builder.add_candidate(mesh, PLA_BASELINE) for mesh in meshes]
    initial = [np.asarray(body.point_set.get_positions()).copy() for body in bodies]
    builder.simulation.run(duration)
    final = [np.asarray(body.point_set.get_positions()).copy() for body in bodies]
    observation = {
        "contract_sha256": contract_sha256(contract),
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
    parser.add_argument("--duration", type=float, default=0.001)
    parser.add_argument("--actuation", action="store_true")
    parser.add_argument("--insertion", action="store_true")
    args = parser.parse_args()
    if args.actuation and args.insertion:
        parser.error("choose only one of --actuation or --insertion")
    (run_insertion if args.insertion else run_actuation if args.actuation else run)(
        args.step.resolve(), args.run_dir.resolve(), args.duration)
