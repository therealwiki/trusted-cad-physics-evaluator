from __future__ import annotations

from dataclasses import dataclass

from .connectors import ConnectorStandard


@dataclass(frozen=True)
class ContactPatch:
    normal_force_n: float
    lever_arm_m: float
    sliding: bool


def contact_torque_capacity(standard: ConnectorStandard, patches: tuple[ContactPatch, ...]) -> float:
    """Coulomb capacity from physical patches; zero patches means zero load path."""
    return sum(standard.friction_coefficient * max(0.0, p.normal_force_n) * max(0.0, p.lever_arm_m)
               for p in patches if not p.sliding)


def transfers_torque_by_contact(standard: ConnectorStandard, applied_torque_nm: float,
                                patches: tuple[ContactPatch, ...], *, has_attachment: bool = False) -> bool:
    if has_attachment:
        raise ValueError("candidate attachment violates the physical interface policy")
    return bool(patches) and applied_torque_nm <= min(standard.max_torque_nm, contact_torque_capacity(standard, patches))
