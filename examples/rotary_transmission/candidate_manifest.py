from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from mechanical_eval.mesh import VolumeMesh


REQUIRED_ROLES = ("input_gear", "output_gear", "lower_bearing_plate", "upper_bearing_plate")


@dataclass(frozen=True)
class ResolvedCandidate:
    by_role: dict[str, VolumeMesh]
    ports_m: dict[str, np.ndarray]
    manifest_path: Path


def load_and_resolve(step_path: Path, meshes: list[VolumeMesh]) -> ResolvedCandidate:
    manifest_path = step_path.with_suffix(".manifest.json")
    if not manifest_path.exists():
        raise ValueError(f"candidate manifest is required: {manifest_path}")
    payload = json.loads(manifest_path.read_text())
    if payload.get("schema_version") != "1.0" or payload.get("step_path") != step_path.name:
        raise ValueError("candidate manifest schema or STEP identity is invalid")
    components = payload.get("components", [])
    roles = [row.get("role") for row in components]
    if sorted(roles) != sorted(REQUIRED_ROLES) or len(meshes) != len(REQUIRED_ROLES):
        raise ValueError(f"candidate must contain exactly the separate roles {REQUIRED_ROLES}")
    expected = np.asarray([row["expected_centroid_mm"] for row in components], dtype=float) * 1e-3
    actual = np.asarray([mesh.vertices_m.mean(axis=0) for mesh in meshes])
    distances = np.linalg.norm(expected[:, None, :] - actual[None, :, :], axis=2)
    by_role: dict[str, VolumeMesh] = {}
    available = set(range(len(meshes)))
    # The expected centers are only selectors; exact STEP coordinates remain authoritative.
    for row_index in np.argsort(distances.min(axis=1)):
        mesh_index = min(available, key=lambda i: distances[row_index, i])
        if distances[row_index, mesh_index] > 5e-3:
            raise ValueError(f"manifest role {roles[row_index]} does not match any STEP solid")
        by_role[str(roles[row_index])] = meshes[mesh_index]
        available.remove(mesh_index)
    ports = payload.get("ports", {})
    if set(ports) != {"input", "output", "mount"}:
        raise ValueError("candidate manifest must declare input, output, and mount ports")
    ports_m = {name: np.asarray(value["origin_mm"], dtype=float) * 1e-3 for name, value in ports.items()}
    return ResolvedCandidate(by_role, ports_m, manifest_path)
