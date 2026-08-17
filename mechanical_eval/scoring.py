from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .schemas import Classification, Evidence, EvidenceSource, EvaluationResult
from .tasks import RotaryTransmissionSpec


@dataclass(frozen=True)
class RotaryObservations:
    input_rotation_rad: float
    output_rotation_rad: float
    input_omega_rad_s: float
    output_omega_rad_s: float
    transmitted_torque_nm: float
    input_stalled: bool
    connector_slip_rad: float
    shaft_wobble_m: float
    component_escape_m: float
    max_strain: float
    input_work_j: float
    output_work_j: float
    solver_converged: bool
    insertion_passed: bool


def score_rotary(spec: RotaryTransmissionSpec, obs: RotaryObservations,
                 evidence: tuple[Evidence, ...]) -> EvaluationResult:
    trusted = {EvidenceSource.PHYSICS_OBSERVATION, EvidenceSource.DERIVED_GEOMETRY,
               EvidenceSource.EXACT_CAD, EvidenceSource.SOLVER}
    if any(e.source not in trusted for e in evidence):
        raise ValueError("declared or synthetic evidence cannot mint physical reward")
    if not obs.solver_converged:
        return EvaluationResult(Classification.INCONCLUSIVE_NUMERICS, 0.0, {}, evidence,
                                ("solver did not converge; no physical failure inferred",))
    ratio = abs(obs.input_omega_rad_s / obs.output_omega_rad_s) if abs(obs.output_omega_rad_s) > 1e-8 else math.inf
    ratio_score = float(np.clip(1 - abs(ratio - spec.target_ratio) / spec.ratio_tolerance, 0, 1))
    direction_score = float(np.sign(obs.input_omega_rad_s * obs.output_omega_rad_s) == spec.expected_direction)
    load_score = float(np.clip(obs.transmitted_torque_nm / spec.output_resistance_nm, 0, 1))
    integrity_score = float(obs.insertion_passed and not obs.input_stalled and obs.connector_slip_rad < 0.1
                            and obs.shaft_wobble_m < 5e-4 and obs.component_escape_m < 1e-3
                            and obs.max_strain < 0.08)
    efficiency = float(np.clip(abs(obs.output_work_j) / max(abs(obs.input_work_j), 1e-9), 0, 1))
    components = {"ratio": ratio_score, "direction": direction_score, "load": load_score,
                  "physical_integrity": integrity_score, "efficiency": efficiency}
    reward = .35*ratio_score + .15*direction_score + .25*load_score + .2*integrity_score + .05*efficiency
    passed = min(ratio_score, direction_score, load_score, integrity_score) >= 0.99
    reasons = () if passed else ("one or more physical acceptance thresholds were not met",)
    return EvaluationResult(Classification.PASS if passed else Classification.FAIL, reward, components, evidence, reasons)
