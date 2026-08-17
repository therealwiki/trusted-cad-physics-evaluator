from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ConnectorStandard:
    id: str
    nominal_diameter_m: float
    flat_depth_m: float
    diametral_clearance_m: float
    engagement_length_m: float
    insertion_speed_m_s: float
    max_insertion_force_n: float
    max_torque_nm: float
    friction_coefficient: float

    @property
    def male_radius_m(self) -> float:
        return self.nominal_diameter_m / 2

    @property
    def female_radius_m(self) -> float:
        return self.male_radius_m + self.diametral_clearance_m / 2


D_SHAFT_6MM_V1 = ConnectorStandard(
    id="d_shaft_6mm_v1", nominal_diameter_m=0.006, flat_depth_m=0.00075,
    diametral_clearance_m=0.00020, engagement_length_m=0.010,
    insertion_speed_m_s=0.002, max_insertion_force_n=35.0,
    max_torque_nm=0.45, friction_coefficient=0.35,
)


@dataclass(frozen=True)
class MountFixture:
    id: str
    bolt_pattern_m: tuple[tuple[float, float], ...]
    bolt_diameter_m: float
    max_load_n: float


MOUNT_30MM_SQUARE_V1 = MountFixture(
    "mount_30mm_square_v1",
    ((-0.015, -0.015), (-0.015, 0.015), (0.015, -0.015), (0.015, 0.015)),
    0.0032, 500.0,
)


@dataclass(frozen=True)
class InsertionObservation:
    inserted: bool
    engagement_m: float
    peak_force_n: float
    interference_m: float
    axial_escape: bool


def validate_insertion(standard: ConnectorStandard, obs: InsertionObservation) -> tuple[bool, tuple[str, ...]]:
    reasons = []
    if not obs.inserted or obs.engagement_m < standard.engagement_length_m:
        reasons.append("insufficient physical engagement")
    if obs.peak_force_n > standard.max_insertion_force_n:
        reasons.append("maximum insertion force exceeded")
    if obs.interference_m > 0:
        reasons.append("connector interference")
    if obs.axial_escape:
        reasons.append("connector escaped axially")
    return not reasons, tuple(reasons)
