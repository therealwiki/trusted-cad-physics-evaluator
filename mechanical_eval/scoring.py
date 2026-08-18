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
    min_deformation_jacobian: float
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
    integrity_score = float(obs.insertion_passed and not obs.input_stalled
                            and obs.connector_slip_rad < spec.connector_slip_rad_max
                            and obs.shaft_wobble_m < spec.shaft_wobble_m_max
                            and obs.component_escape_m < spec.component_escape_m_max
                            and obs.max_strain < spec.max_strain
                            and obs.min_deformation_jacobian > spec.min_deformation_jacobian)
    efficiency = float(np.clip(abs(obs.output_work_j) / max(abs(obs.input_work_j), 1e-9), 0, 1))
    components = {"ratio": ratio_score, "direction": direction_score, "load": load_score,
                  "physical_integrity": integrity_score, "efficiency": efficiency}
    reward = .35*ratio_score + .15*direction_score + .25*load_score + .2*integrity_score + .05*efficiency
    passed = (ratio_score >= spec.ratio_score_min and direction_score == 1.0
              and load_score >= spec.load_score_min and integrity_score == 1.0)
    reasons = () if passed else ("one or more physical acceptance thresholds were not met",)
    return EvaluationResult(Classification.PASS if passed else Classification.FAIL, reward, components, evidence, reasons)
