from __future__ import annotations

from dataclasses import dataclass

from .connectors import D_SHAFT_6MM_V1, MOUNT_30MM_SQUARE_V1
from .schemas import SimulationSettings, TaskSpec


@dataclass(frozen=True)
class RotaryTransmissionSpec:
    target_ratio: float = 2.0
    ratio_tolerance: float = 0.15
    expected_direction: int = -1
    input_speed_rad_s: float = 6.0
    input_torque_limit_nm: float = 0.25
    output_resistance_nm: float = 0.08
    duration_s: float = 3.0
    connector_slip_rad_max: float = 0.1
    shaft_wobble_m_max: float = 5e-4
    component_escape_m_max: float = 1e-3
    max_strain: float = 0.02
    min_deformation_jacobian: float = 0.0
    ratio_score_min: float = 0.99
    load_score_min: float = 0.99

    def __post_init__(self) -> None:
        if self.target_ratio <= 0 or self.ratio_tolerance <= 0 or self.expected_direction not in {-1, 1}:
            raise ValueError("invalid rotary benchmark targets")
        if self.input_torque_limit_nm > D_SHAFT_6MM_V1.max_torque_nm:
            raise ValueError("actuation exceeds connector load limit")

    def task_spec(self) -> TaskSpec:
        return TaskSpec(
            task_type="rotary_transmission_v1", required_ports=("input", "output", "mount"),
            connector_standards={"input": D_SHAFT_6MM_V1.id, "output": D_SHAFT_6MM_V1.id,
                                 "mount": MOUNT_30MM_SQUARE_V1.id},
            settings=SimulationSettings(duration_s=self.duration_s),
        )
