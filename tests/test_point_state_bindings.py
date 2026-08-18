import numpy as np
import pystark


def test_bulk_point_state_bindings_return_stark_state(tmp_path):
    settings = pystark.Settings()
    settings.output.enable_output = False
    settings.output.simulation_name = "point_state_bindings"
    settings.output.output_directory = str(tmp_path / "output")
    settings.output.codegen_directory = str(tmp_path / "codegen")
    settings.simulation.init_frictional_contact = False
    simulation = pystark.Simulation(settings)
    _, _, handler = simulation.presets().deformables().add_volume_grid(
        "tet_grid", np.ones(3) * 0.01, np.ones(3, dtype=np.int32), pystark.Volume.Params.Soft_Rubber()
    )
    points = handler.point_set
    assert points.get_positions().shape == (points.size(), 3)
    assert points.get_rest_positions().shape == (points.size(), 3)
    assert points.get_velocities().shape == (points.size(), 3)
    assert points.get_forces().shape == (points.size(), 3)
