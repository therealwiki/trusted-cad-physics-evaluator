from mechanical_eval.contract import contract_sha256, load_contract


def test_prelock_validation_contract_has_stable_audit_hash():
    contract = load_contract("examples/rotary_transmission/contracts/rotary_transmission_v1.json",
                             allow_prelock=True)
    assert contract["contract_id"] == "rotary_transmission_v1"
    assert contract["contract_version"] == "1.25.0"
    assert contract["frozen_before_attempts"] is False
    assert contract_sha256(contract) == "a4d8150356ddaeed36ceb53e84f941db7a3b03c877ced9f6acf0355511b01d6d"


def test_contract_records_anti_cheating_boundary():
    policy = load_contract("examples/rotary_transmission/contracts/rotary_transmission_v1.json",
                           allow_prelock=True)["candidate_policy"]
    assert policy["components"] == "separate_tetrahedral_fem_deformables"
    assert not any((policy["candidate_attachments"], policy["candidate_constraints"],
                    policy["candidate_prescribed_motion"], policy["candidate_direct_forces"]))
