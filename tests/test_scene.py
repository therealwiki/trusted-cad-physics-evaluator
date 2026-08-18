import numpy as np

from mechanical_eval.mesh import VolumeMesh, tet_surface
from mechanical_eval.scene import StarkSceneBuilder
from mechanical_eval.schemas import SimulationSettings


def test_candidate_is_real_stark_deformable_with_no_hidden_constraint(tmp_path):
    vertices = np.array([[0., 0., 0.], [.001, 0, 0], [0, .001, 0], [0, 0, .001]])
    tets = np.array([[0, 1, 2, 3]], dtype=np.int32)
    mesh = VolumeMesh("candidate", vertices, tets, tet_surface(tets))
    scene = StarkSceneBuilder(SimulationSettings(duration_s=.001,
                                                  ipc_min_contact_stiffness=2e8,
                                                  ipc_max_contact_stiffness=3e11), tmp_path)
    body = scene.add_candidate(mesh)
    assert body.point_set.size() == 4
    contact = scene.simulation.interactions().contact().get_global_params()
    assert contact.default_contact_thickness == 2.5e-5
    assert contact.min_contact_stiffness == 2e8
    assert contact.max_contact_stiffness == 3e11
    assert contact.friction_stick_slide_threshold == 1e-4
    before = scene.simulation.get_time()
    assert scene.simulation.run_one_time_step() is True
    assert scene.simulation.get_time() > before
    audit = scene.policy_audit()
    assert audit["audit_method"] == "candidate_construction_ledger_v1"
    assert audit["candidate_count"] == 1
    assert audit["candidate_is_deformable"] is True
    assert audit["candidate_attachments"] is False
    assert audit["candidate_internal_constraints"] is False
    assert audit["candidate_prescribed_motion"] is False
    assert audit["candidate_direct_forces"] is False
    assert len(audit["ledger_sha256"]) == 64
