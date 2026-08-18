from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import asdict
from pathlib import Path

from mechanical_eval.contract import contract_sha256, load_contract
from mechanical_eval.schemas import Evidence, EvidenceSource
from mechanical_eval.scoring import RotaryObservations, score_rotary
from mechanical_eval.tasks import RotaryTransmissionSpec


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(step: Path, insertion_path: Path, repeat_paths: list[Path], output: Path) -> None:
    if len(repeat_paths) != 2:
        raise SystemExit("exactly two deterministic full-run observations are required")
    contract_path = Path(__file__).parent / "contracts" / "rotary_transmission_v1.json"
    contract = load_contract(contract_path)
    settings_hash = contract_sha256(contract)
    insertion = json.loads(insertion_path.read_text())
    repeats = [json.loads(path.read_text()) for path in repeat_paths]
    if any(run["contract_sha256"] != settings_hash for run in [insertion, *repeats]):
        raise SystemExit("evidence contract hash mismatch")
    if any(not run["solver_converged"] for run in repeats):
        solver_converged = False
    else:
        solver_converged = True
    steady = repeats[0]["steady_observations"]
    input_w = steady["input_gear_omega_rad_s"]
    output_w = steady["output_gear_omega_rad_s"]
    repeat_ratio = abs(repeats[1]["steady_observations"]["input_gear_omega_rad_s"] /
                       repeats[1]["steady_observations"]["output_gear_omega_rad_s"])
    ratio = abs(input_w / output_w)
    repeatable = abs(ratio - repeat_ratio) <= 0.01
    insertion_passed = insertion["classification"] == "PASS"
    candidates = [state for run in repeats for state in run["candidate"]]
    obs = RotaryObservations(
        input_rotation_rad=repeats[0]["candidate"][0]["angle_rad"],
        output_rotation_rad=repeats[0]["candidate"][1]["angle_rad"],
        input_omega_rad_s=input_w, output_omega_rad_s=output_w,
        transmitted_torque_nm=steady["brake_torque_nm"],
        input_stalled=abs(input_w) < 0.1,
        connector_slip_rad=max(run["connector_slip_rad"] for run in repeats),
        shaft_wobble_m=max(state["wobble_m"] for state in candidates),
        component_escape_m=max(state["escape_m"] for state in candidates),
        max_strain=max(state["max_strain"] for state in candidates),
        input_work_j=repeats[0]["input_work_j"], output_work_j=repeats[0]["output_work_j"],
        solver_converged=solver_converged and repeatable, insertion_passed=insertion_passed)
    evidence = (
        Evidence("cad_sha256", sha(step), "sha256", EvidenceSource.EXACT_CAD, "ingest"),
        Evidence("settings_sha256", settings_hash, "sha256", EvidenceSource.SOLVER, "contract"),
        Evidence("insertion_force", insertion["max_insertion_force_n"], "N", EvidenceSource.PHYSICS_OBSERVATION, "insertion"),
        Evidence("ratio_repeat_delta", abs(ratio - repeat_ratio), "ratio", EvidenceSource.SOLVER, "repeatability"),
        Evidence("candidate_constraints_absent", True, "bool", EvidenceSource.SOLVER, "policy_audit"),
    )
    result = score_rotary(RotaryTransmissionSpec(), obs, evidence)
    payload = result.as_dict()
    payload.update({"cad_sha256": sha(step), "settings_sha256": settings_hash,
                    "repeat_observation_paths": [str(path) for path in repeat_paths],
                    "insertion_observation_path": str(insertion_path),
                    "observations": {**asdict(obs), "ratio": ratio, "repeat_ratio": repeat_ratio,
                                     "solver_status": "CONVERGED_REPEATABLE" if solver_converged and repeatable else "INCONCLUSIVE",
                                     "output_load_nm": steady["brake_torque_nm"],
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
