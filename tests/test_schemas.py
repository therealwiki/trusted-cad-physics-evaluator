import numpy as np
import pytest

from mechanical_eval.schemas import Frame, Port, SimulationSettings, SubmissionManifest, TaskSpec


def test_frame_transform_preserves_rotation_and_translation():
    frame = Frame((1.0, 2.0, 3.0), (0.0, 0.0, np.sqrt(0.5), np.sqrt(0.5)))
    np.testing.assert_allclose(frame.matrix() @ [1, 0, 0, 1], [1, 3, 3, 1], atol=1e-12)


def test_manifest_rejects_duplicate_ports_and_non_step():
    p = Port("input", Frame((0, 0, 0)), "d")
    with pytest.raises(ValueError):
        SubmissionManifest("part.step", (p, p))
    with pytest.raises(ValueError):
        SubmissionManifest("part.stl", (p,))


def test_settings_make_contact_scale_explicit_and_small():
    with pytest.raises(ValueError):
        SimulationSettings(surface_size_m=1e-4, contact_distance_m=2e-4)
    with pytest.raises(ValueError):
        SimulationSettings(ipc_min_contact_stiffness=2e8, ipc_max_contact_stiffness=1e8)


def test_task_requires_prescribed_standard_for_each_port():
    with pytest.raises(ValueError):
        TaskSpec("x", ("a", "b"), {"a": "standard"})
