import numpy as np

from mechanical_eval.mesh import VolumeMesh, tet_surface
from mechanical_eval.scene import StarkSceneBuilder
from mechanical_eval.schemas import SimulationSettings


def test_candidate_is_real_stark_deformable_with_no_hidden_constraint(tmp_path):
    vertices = np.array([[0., 0., 0.], [.001, 0, 0], [0, .001, 0], [0, 0, .001]])
    tets = np.array([[0, 1, 2, 3]], dtype=np.int32)
    mesh = VolumeMesh("candidate", vertices, tets, tet_surface(tets))
    scene = StarkSceneBuilder(SimulationSettings(duration_s=.001), tmp_path)
    body = scene.add_candidate(mesh)
    assert body.point_set.size() == 4
    assert scene.policy_audit() == {
        "candidate_is_deformable": True,
        "candidate_attachments": False,
        "candidate_internal_constraints": False,
        "candidate_prescribed_motion": False,
    }
