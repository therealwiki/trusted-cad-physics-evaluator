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
    assert scene.policy_audit() == {
        "candidate_is_deformable": True,
        "candidate_attachments": False,
        "candidate_internal_constraints": False,
        "candidate_prescribed_motion": False,
    }
