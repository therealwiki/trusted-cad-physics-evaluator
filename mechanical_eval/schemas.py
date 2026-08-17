from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import StrEnum
from pathlib import Path
from typing import Any, Literal

import numpy as np

SCHEMA_VERSION = "1.0"


def _positive(name: str, value: float) -> None:
    if not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and > 0")


@dataclass(frozen=True)
class Frame:
    """Right-handed datum frame, expressed in assembly coordinates (metres)."""

    origin_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        q = np.asarray(self.quaternion_xyzw, dtype=float)
        if len(self.origin_m) != 3 or not np.all(np.isfinite(self.origin_m)):
            raise ValueError("origin_m must contain three finite SI coordinates")
        if q.shape != (4,) or not np.all(np.isfinite(q)) or not np.isclose(np.linalg.norm(q), 1, atol=1e-6):
            raise ValueError("quaternion_xyzw must be a finite unit quaternion")

    def matrix(self) -> np.ndarray:
        x, y, z, w = self.quaternion_xyzw
        r = np.array([
            [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
            [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
            [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
        ])
        out = np.eye(4)
        out[:3, :3] = r
        out[:3, 3] = self.origin_m
        return out


@dataclass(frozen=True)
class Port:
    name: str
    frame: Frame
    connector_standard: str

    def __post_init__(self) -> None:
        if not self.name or not self.connector_standard:
            raise ValueError("port name and connector_standard are required")


@dataclass(frozen=True)
class SubmissionManifest:
    step_path: str
    ports: tuple[Port, ...]
    schema_version: Literal["1.0"] = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if Path(self.step_path).suffix.lower() not in {".step", ".stp"}:
            raise ValueError("step_path must identify a STEP file")
        names = [p.name for p in self.ports]
        if len(names) != len(set(names)):
            raise ValueError("port names must be unique")


@dataclass(frozen=True)
class SimulationSettings:
    surface_size_m: float = 8e-4
    volume_size_m: float = 1.2e-3
    contact_distance_m: float = 5e-5
    time_step_s: float = 1 / 1000
    duration_s: float = 1.0
    friction_coefficient: float = 0.35
    schema_version: Literal["1.0"] = SCHEMA_VERSION

    def __post_init__(self) -> None:
        for name in ("surface_size_m", "volume_size_m", "contact_distance_m", "time_step_s", "duration_s"):
            _positive(name, getattr(self, name))
        if self.contact_distance_m >= self.surface_size_m:
            raise ValueError("contact distance must be smaller than surface mesh size")
        if not 0 <= self.friction_coefficient <= 2:
            raise ValueError("friction_coefficient must be in [0, 2]")


@dataclass(frozen=True)
class TaskSpec:
    task_type: str
    required_ports: tuple[str, ...]
    connector_standards: dict[str, str]
    settings: SimulationSettings = field(default_factory=SimulationSettings)
    schema_version: Literal["1.0"] = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if not self.task_type or not self.required_ports:
            raise ValueError("task_type and required_ports are required")
        if set(self.required_ports) != set(self.connector_standards):
            raise ValueError("every required port must have exactly one prescribed connector standard")


class Classification(StrEnum):
    PASS = "PASS"
    FAIL = "FAIL"
    INCONCLUSIVE_NUMERICS = "INCONCLUSIVE_NUMERICS"


class EvidenceSource(StrEnum):
    EXACT_CAD = "EXACT_CAD"
    DERIVED_GEOMETRY = "DERIVED_GEOMETRY"
    PHYSICS_OBSERVATION = "PHYSICS_OBSERVATION"
    SOLVER = "SOLVER"
    DECLARED = "DECLARED"
    SYNTHETIC = "SYNTHETIC"


@dataclass(frozen=True)
class Evidence:
    metric: str
    value: float | int | bool | str
    unit: str
    source: EvidenceSource
    phase: str


@dataclass(frozen=True)
class EvaluationResult:
    classification: Classification
    reward: float
    reward_components: dict[str, float]
    evidence: tuple[Evidence, ...]
    reasons: tuple[str, ...] = ()
    schema_version: Literal["1.0"] = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if not 0 <= self.reward <= 1 or any(not 0 <= v <= 1 for v in self.reward_components.values()):
            raise ValueError("reward and components must lie in [0, 1]")

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)
