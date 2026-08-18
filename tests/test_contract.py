from mechanical_eval.contract import contract_sha256, load_contract


def test_prelock_validation_contract_has_stable_audit_hash():
    contract = load_contract("examples/rotary_transmission/contracts/rotary_transmission_v1.json",
                             allow_prelock=True)
    assert contract["contract_id"] == "rotary_transmission_v1"
    assert contract["contract_version"] == "1.11.0"
    assert contract["frozen_before_attempts"] is False
    assert contract_sha256(contract) == "23b961c5554e1610d13b38e14a6db27b24d1f37677b698cf6725fbbcbd6b71d9"


def test_contract_records_anti_cheating_boundary():
    policy = load_contract("examples/rotary_transmission/contracts/rotary_transmission_v1.json",
                           allow_prelock=True)["candidate_policy"]
    assert policy["components"] == "separate_tetrahedral_fem_deformables"
    assert not any((policy["candidate_attachments"], policy["candidate_constraints"],
                    policy["candidate_prescribed_motion"], policy["candidate_direct_forces"]))
