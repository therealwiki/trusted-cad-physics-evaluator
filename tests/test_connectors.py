import pytest

from mechanical_eval.connectors import D_SHAFT_6MM_V1, InsertionObservation, validate_insertion
from mechanical_eval.contact_path import ContactPatch, transfers_torque_by_contact


def test_insertion_is_physical_and_limited():
    ok = InsertionObservation(True, 0.010, 12.0, 0.0, False)
    assert validate_insertion(D_SHAFT_6MM_V1, ok) == (True, ())
    bad = InsertionObservation(True, 0.002, 50.0, 0.0001, True)
    passed, reasons = validate_insertion(D_SHAFT_6MM_V1, bad)
    assert not passed and len(reasons) == 4


def test_torque_path_requires_contact_and_never_attachment():
    patches = (ContactPatch(100.0, 0.0025, False), ContactPatch(100.0, 0.0025, False))
    assert transfers_torque_by_contact(D_SHAFT_6MM_V1, 0.10, patches)
    assert not transfers_torque_by_contact(D_SHAFT_6MM_V1, 0.10, ())
    with pytest.raises(ValueError, match="attachment"):
        transfers_torque_by_contact(D_SHAFT_6MM_V1, 0.10, patches, has_attachment=True)
