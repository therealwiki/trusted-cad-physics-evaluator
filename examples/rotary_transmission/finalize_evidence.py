from __future__ import annotations

import argparse
import hashlib
import json
import math
from dataclasses import asdict
from pathlib import Path

from mechanical_eval.contract import contract_sha256, load_contract
from mechanical_eval.schemas import Evidence, EvidenceSource
from mechanical_eval.scoring import RotaryObservations, score_rotary
from mechanical_eval.tasks import RotaryTransmissionSpec


def _current_evaluator_source_hashes() -> dict[str, str]:
    root = Path(__file__).parents[2]
    files = (
        Path(__file__).parent / "run_harness.py", Path(__file__).parent / "actuation_scene.py",
        Path(__file__).parent / "insertion_scene.py", Path(__file__).parent / "candidate_manifest.py",
        root / "mechanical_eval" / "scene.py", root / "mechanical_eval" / "scoring.py",
        root / "stark" / "extern" / "symx" / "src" / "solver" / "NewtonsMethod.cpp",
    )
    return {str(path.relative_to(root)): sha(path) for path in files}


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _require_finite_numbers(value: object, path: str = "evidence") -> None:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            raise SystemExit(f"non-finite numeric evidence at {path}")
        return
    if isinstance(value, dict):
        for key, child in value.items():
            _require_finite_numbers(child, f"{path}.{key}")
    elif isinstance(value, list):
        for index, child in enumerate(value):
            _require_finite_numbers(child, f"{path}[{index}]")


def main(step: Path, insertion_path: Path, repeat_paths: list[Path], output: Path) -> None:
    if len(repeat_paths) != 2:
        raise SystemExit("exactly two deterministic full-run observations are required")
    contract_path = Path(__file__).parent / "contracts" / "rotary_transmission_v1.json"
    contract = load_contract(contract_path)
    settings_hash = contract_sha256(contract)
    cad_hash = sha(step)
    if len({path.resolve() for path in repeat_paths}) != 2:
        raise SystemExit("repeat observations must be distinct run artifacts")
    insertion = json.loads(insertion_path.read_text())
    repeats = [json.loads(path.read_text()) for path in repeat_paths]
    _require_finite_numbers(insertion, "insertion")
    for index, repeat in enumerate(repeats):
        _require_finite_numbers(repeat, f"repeat[{index}]")
    current_sources = _current_evaluator_source_hashes()
    artifact_paths = [insertion_path, *repeat_paths]
    manifests = [json.loads((path.parent / "run_manifest.json").read_text()) for path in artifact_paths]
    if any(manifest.get("evaluator_source_sha256") != current_sources for manifest in manifests):
        raise SystemExit("evaluator source hash mismatch; evidence was not produced by the reviewed evaluator")
    if any(run["contract_sha256"] != settings_hash for run in [insertion, *repeats]):
        raise SystemExit("evidence contract hash mismatch")
    if any(run.get("cad_sha256") != cad_hash for run in [insertion, *repeats]):
        raise SystemExit("evidence CAD hash mismatch")
    expected_duration = contract["actuation"]["actuation_duration_s"]
    expected_steps = round(expected_duration / contract["mesh_and_contact"]["fixed_time_step_s"])
    if any(abs(run["duration_s"] - expected_duration) > 1e-12 or
           run["completed_steps"] != expected_steps for run in repeats):
        raise SystemExit("repeat is not a complete locked-duration run")
    insertion_dt = contract["mesh_and_contact"]["fixed_time_step_s"]
    if (insertion.get("completed_steps") != round(insertion["duration_s"] / insertion_dt) or
            insertion["duration_s"] + 1e-12 <
            (contract["rig_geometry"]["initial_insertion_gap_m"] +
             contract["connector"]["engagement_length_m"]) /
            contract["connector"]["insertion_speed_m_s"]):
        raise SystemExit("insertion is not a complete required-travel run")
    if any(len(run.get("candidate", [])) != 4 for run in repeats):
        raise SystemExit("full candidate assembly must contain four separately observed deformable bodies")
    required_policy = {"candidate_is_deformable": True, "candidate_attachments": False,
                       "candidate_internal_constraints": False, "candidate_prescribed_motion": False,
                       "candidate_direct_forces": False,
                       "audit_method": "candidate_construction_ledger_v1"}
    if any(any(run.get("policy_audit", {}).get(key) != value for key, value in required_policy.items())
           for run in [insertion, *repeats]):
        raise SystemExit("candidate policy audit missing or failed")
    if any(run.get("max_motor_torque_nm", float("inf")) > contract["actuation"]["input_torque_limit_nm"] + 1e-12
           for run in repeats):
        raise SystemExit("motor torque limit was exceeded")
    solver_converged = all(run["solver_converged"] for run in repeats)
    steadies = [run["steady_observations"] for run in repeats]
    ratios = [abs(row["input_gear_omega_rad_s"] / row["output_gear_omega_rad_s"])
              if abs(row["output_gear_omega_rad_s"]) > 1e-8 else float("inf") for row in steadies]
    worst_ratio_index = max(range(2), key=lambda i: abs(ratios[i] - contract["acceptance"]["target_ratio"]))
    input_w = steadies[worst_ratio_index]["input_gear_omega_rad_s"]
    output_w = steadies[worst_ratio_index]["output_gear_omega_rad_s"]
    ratio, repeat_ratio = ratios
    omega_scale = max(abs(steadies[0]["input_gear_omega_rad_s"]),
                      abs(steadies[1]["input_gear_omega_rad_s"]), 1e-8)
    repeatable = (abs(ratio - repeat_ratio) <= contract["acceptance"]["repeat_ratio_delta_max"] and
                  abs(steadies[0]["input_gear_omega_rad_s"] - steadies[1]["input_gear_omega_rad_s"]) /
                  omega_scale <= contract["acceptance"]["repeat_omega_relative_delta_max"] and
                  abs(steadies[0]["transmitted_contact_torque_nm"] -
                      steadies[1]["transmitted_contact_torque_nm"]) <=
                  contract["acceptance"]["repeat_load_delta_nm_max"])
    insertion_passed = (insertion.get("solver_converged") and insertion["classification"] == "PASS" and
                        min(insertion.get("measured_engagement_m", [0.0])) >=
                        contract["connector"]["engagement_length_m"] and
                        insertion["max_insertion_force_n"] < contract["connector"]["max_insertion_force_n"])
    candidates = [state for run in repeats for state in run["candidate"]]
    spec = RotaryTransmissionSpec(
        target_ratio=contract["acceptance"]["target_ratio"],
        ratio_tolerance=contract["acceptance"]["ratio_tolerance"],
        expected_direction=contract["acceptance"]["expected_direction"],
        input_speed_rad_s=contract["actuation"]["input_speed_target_rad_s"],
        input_torque_limit_nm=contract["actuation"]["input_torque_limit_nm"],
        output_resistance_nm=contract["actuation"]["output_resistance_nm"],
        duration_s=expected_duration,
        connector_slip_rad_max=contract["acceptance"]["connector_slip_rad_max_exclusive"],
        shaft_wobble_m_max=contract["acceptance"]["shaft_wobble_m_max_exclusive"],
        component_escape_m_max=contract["acceptance"]["component_escape_m_max_exclusive"],
        max_strain=contract["acceptance"]["max_strain_max_exclusive"],
        min_deformation_jacobian=contract["acceptance"]["min_deformation_jacobian_exclusive"],
        ratio_score_min=contract["acceptance"]["ratio_score_min"],
        load_score_min=contract["acceptance"]["load_score_min"],
    )
    obs = RotaryObservations(
        input_rotation_rad=repeats[0]["candidate"][0]["angle_rad"],
        output_rotation_rad=repeats[0]["candidate"][1]["angle_rad"],
        input_omega_rad_s=input_w, output_omega_rad_s=output_w,
        transmitted_torque_nm=min(row["transmitted_contact_torque_nm"] for row in steadies),
        input_stalled=any(abs(row["input_gear_omega_rad_s"]) < 0.1 for row in steadies),
        connector_slip_rad=max(run["connector_slip_rad"] for run in repeats),
        shaft_wobble_m=max(state["wobble_m"] for state in candidates),
        component_escape_m=max(state["escape_m"] for state in candidates),
        max_strain=max(state["max_strain"] for state in candidates),
        min_deformation_jacobian=min(state["min_deformation_jacobian"] for state in candidates),
        input_work_j=max(run["input_work_j"] for run in repeats),
        output_work_j=min(run["output_work_j"] for run in repeats),
        solver_converged=solver_converged and repeatable, insertion_passed=insertion_passed)
    evidence = (
        Evidence("cad_sha256", sha(step), "sha256", EvidenceSource.EXACT_CAD, "ingest"),
        Evidence("settings_sha256", settings_hash, "sha256", EvidenceSource.SOLVER, "contract"),
        Evidence("insertion_force", insertion["max_insertion_force_n"], "N", EvidenceSource.PHYSICS_OBSERVATION, "insertion"),
        Evidence("ratio_repeat_delta", abs(ratio - repeat_ratio), "ratio", EvidenceSource.SOLVER, "repeatability"),
        Evidence("candidate_constraints_absent", True, "bool", EvidenceSource.SOLVER, "policy_audit"),
    )
    result = score_rotary(spec, obs, evidence)
    payload = result.as_dict()
    payload.update({"cad_sha256": sha(step), "settings_sha256": settings_hash,
                    "repeat_observation_paths": [str(path) for path in repeat_paths],
                    "insertion_observation_path": str(insertion_path),
                    "observations": {**asdict(obs), "ratio": ratio, "repeat_ratio": repeat_ratio,
                                     "solver_status": "CONVERGED_REPEATABLE" if solver_converged and repeatable else "INCONCLUSIVE",
                                     "output_load_nm": obs.transmitted_torque_nm,
                                     "max_wobble_m": obs.shaft_wobble_m}})
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(payload, indent=2, default=str) + "\n")
    if payload["classification"] != "PASS":
        raise SystemExit(f"final physical result is {payload['classification']}; no PASS minted")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=Path)
    parser.add_argument("insertion", type=Path)
    parser.add_argument("repeat1", type=Path)
    parser.add_argument("repeat2", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    main(args.step.resolve(), args.insertion.resolve(), [args.repeat1.resolve(), args.repeat2.resolve()], args.output.resolve())
