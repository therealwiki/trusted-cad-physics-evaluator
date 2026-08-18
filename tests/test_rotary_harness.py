import importlib.util
from pathlib import Path

import numpy as np
import pytest
import trimesh

from mechanical_eval.contract import load_contract


def _module():
    path = Path(__file__).parents[1] / "examples" / "rotary_transmission" / "run_harness.py"
    spec = importlib.util.spec_from_file_location("rotary_run_harness", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _rig_geometry():
    path = Path(__file__).parents[1] / "examples" / "rotary_transmission" / "rig_geometry.py"
    spec = importlib.util.spec_from_file_location("rotary_rig_geometry", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class _Contact:
    def __init__(self):
        self.stiffness = 1e8

    def get_contact_stiffness(self):
        return self.stiffness


class _Interactions:
    def __init__(self, contact):
        self._contact = contact

    def contact(self):
        return self._contact


class _RetryingSimulation:
    def __init__(self):
        self.time = 0.0
        self.contact = _Contact()
        self.calls = 0

    def get_time(self):
        return self.time

    def interactions(self):
        return _Interactions(self.contact)

    def run_one_time_step(self):
        self.calls += 1
        if self.calls == 1:
            self.contact.stiffness *= 2
        else:
            self.time += 0.001


def test_contact_hardening_retries_same_physical_step():
    simulation = _RetryingSimulation()
    advanced, retries = _module()._advance_with_contact_retries(simulation, 1e12)
    assert advanced is True
    assert retries == 1
    assert simulation.calls == 2
    assert simulation.time == 0.001


def test_locked_insertion_requires_gap_plus_engagement_travel():
    contract = load_contract(
        Path(__file__).parents[1] / "examples" / "rotary_transmission" / "contracts" /
        "rotary_transmission_v1.json", allow_prelock=True)
    assert _module()._minimum_insertion_duration(contract) == 5.5


def test_evaluator_contact_meshes_are_closed_and_outward_oriented():
    geometry = _rig_geometry()
    for vertices, triangles in (geometry.d_shaft_mesh(), geometry.sleeve_mesh()):
        mesh = trimesh.Trimesh(vertices, triangles, process=False)
        assert mesh.is_watertight
        assert mesh.is_winding_consistent
        assert mesh.volume > 0
        assert np.all(np.linalg.eigvalsh(geometry.solid_inertia(vertices, triangles, 0.05)) > 0)


def test_run_directory_is_append_only_and_snapshots_contract(tmp_path):
    harness = _module()
    step = tmp_path / "candidate.step"
    step.write_bytes(b"STEP")
    contract = load_contract(
        Path(__file__).parents[1] / "examples" / "rotary_transmission" / "contracts" /
        "rotary_transmission_v1.json", allow_prelock=True)
    run_dir = tmp_path / "run"
    harness._prepare_run_dir(run_dir, contract, step)
    assert (run_dir / "contract_snapshot.json").exists()
    assert (run_dir / "run_manifest.json").exists()
    with pytest.raises(FileExistsError, match="immutable run directory"):
        harness._prepare_run_dir(run_dir, contract, step)
