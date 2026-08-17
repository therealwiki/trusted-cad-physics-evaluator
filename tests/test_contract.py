from mechanical_eval.contract import contract_sha256, load_contract


def test_frozen_contract_has_stable_canonical_hash():
    contract = load_contract("contracts/rotary_transmission_v1.json")
    assert contract["contract_id"] == "rotary_transmission_v1"
    assert contract_sha256(contract) == "b0a972448234c8484d5d3db3499b01916d130404604cd84ca247e18212c724df"


def test_contract_records_anti_cheating_boundary():
    policy = load_contract("contracts/rotary_transmission_v1.json")["candidate_policy"]
    assert policy["components"] == "separate_tetrahedral_fem_deformables"
    assert not any((policy["candidate_attachments"], policy["candidate_constraints"],
                    policy["candidate_prescribed_motion"], policy["candidate_direct_forces"]))
