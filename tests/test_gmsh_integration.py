from pathlib import Path

import gmsh
import pytest

from mechanical_eval.mesh import GmshOccBackend


def write_two_solid_step(path: Path) -> None:
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.add("two_solids")
        gmsh.model.occ.addBox(0, 0, 0, 4, 4, 4)
        gmsh.model.occ.addBox(8, 0, 0, 3, 3, 3)
        gmsh.model.occ.synchronize()
        gmsh.write(str(path))
    finally:
        gmsh.finalize()


@pytest.mark.integration
def test_step_preserves_separate_solids_and_tetrahedralizes_each(tmp_path):
    step = tmp_path / "assembly.step"
    write_two_solid_step(step)
    meshes = GmshOccBackend().mesh_step(step, size_m=0.002, source_units="mm")
    assert len(meshes) == 2
    assert all(len(mesh.tets) and len(mesh.surface_triangles) for mesh in meshes)
    assert sorted(round(mesh.volume_m3 * 1e9) for mesh in meshes) == [27, 64]
