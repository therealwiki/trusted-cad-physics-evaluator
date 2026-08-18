import numpy as np
import pytest

from mechanical_eval.mesh import VolumeMesh, initial_overlap_pairs, tet_surface


def tetra(offset=(0, 0, 0)):
    vertices = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [0, 0, 1]], dtype=float) + offset
    tets = np.array([[0, 1, 2, 3]], dtype=np.int32)
    return VolumeMesh("tet", vertices, tets, tet_surface(tets))


def test_tet_volume_surface_and_mass_are_geometry_derived():
    mesh = tetra()
    mesh.validate()
    assert mesh.volume_m3 == 1 / 6
    assert len(mesh.surface_triangles) == 4
    mass, center, inertia = mesh.mass_properties(6.0)
    assert mass == 1.0
    assert center.shape == (3,)
    assert inertia.shape == (3, 3)


def test_component_separation_and_overlap_broad_phase():
    assert initial_overlap_pairs([tetra(), tetra((2, 0, 0))]) == []
    assert initial_overlap_pairs([tetra(), tetra((0.5, 0, 0))]) == [(0, 1)]


def test_mesh_validation_rejects_inverted_tetrahedra():
    mesh = tetra()
    inverted = VolumeMesh(mesh.name, mesh.vertices_m, mesh.tets[:, [0, 2, 1, 3]], mesh.surface_triangles)
    with pytest.raises(ValueError, match="inverted or degenerate"):
        inverted.validate()
