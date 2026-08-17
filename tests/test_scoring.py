import pytest

from mechanical_eval.schemas import Classification, Evidence, EvidenceSource
from mechanical_eval.scoring import RotaryObservations, score_rotary
from mechanical_eval.tasks import RotaryTransmissionSpec


def observations(**updates):
    values = dict(input_rotation_rad=6, output_rotation_rad=-3, input_omega_rad_s=6,
                  output_omega_rad_s=-3, transmitted_torque_nm=.08, input_stalled=False,
                  connector_slip_rad=.01, shaft_wobble_m=1e-4, component_escape_m=0,
                  max_strain=.02, input_work_j=.1, output_work_j=.08,
                  solver_converged=True, insertion_passed=True)
    values.update(updates)
    return RotaryObservations(**values)


def test_physical_observations_can_pass_with_visible_components():
    evidence = (Evidence("output_omega", -3.0, "rad/s", EvidenceSource.PHYSICS_OBSERVATION, "actuation"),)
    result = score_rotary(RotaryTransmissionSpec(), observations(), evidence)
    assert result.classification == Classification.PASS
    assert set(result.reward_components) == {"ratio", "direction", "load", "physical_integrity", "efficiency"}


def test_numerical_failure_is_never_a_physical_jam():
    result = score_rotary(RotaryTransmissionSpec(), observations(solver_converged=False), ())
    assert result.classification == Classification.INCONCLUSIVE_NUMERICS


@pytest.mark.parametrize("source", [EvidenceSource.DECLARED, EvidenceSource.SYNTHETIC])
def test_untrusted_trace_cannot_mint_reward(source):
    evidence = (Evidence("ratio", 2.0, "1", source, "actuation"),)
    with pytest.raises(ValueError, match="cannot mint"):
        score_rotary(RotaryTransmissionSpec(), observations(), evidence)
