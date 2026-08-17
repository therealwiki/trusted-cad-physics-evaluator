# Mechanical CAD evaluator foundation

`mechanical_eval` replaces the former rigid `rlgear` prototype. There is no
legacy hinge-based reward path in the package. A submission supplies STEP plus
datum frames; those frames locate trusted fixtures and never attach them.

## Trusted stages

1. Validate the versioned manifest, SI units, required ports, and evaluator-
   prescribed connector standards.
2. Import STEP with Gmsh/OpenCASCADE and retain each OCC volume as a separate
   printable component.
3. Generate a separate linear tetrahedral mesh and derived boundary surface for
   every component. Independently recompute volume, mass, centre of mass,
   inertia, bounds, and conservative initial-overlap candidates.
4. Build each candidate component as a STARK tetrahedral FEM volume with IPC
   surface contact. The scene policy prohibits candidate attachments, joints,
   and prescribed motion.
5. Insert evaluator-owned connectors, settle, apply torque-limited actuation and
   physical output resistance, observe response, and score trusted evidence.
6. Return `PASS`, `FAIL`, or `INCONCLUSIVE_NUMERICS`. Solver failure never means
   physical jam, and declared/synthetic evidence is rejected by the scorer.

The backend boundary is `MeshingBackend`; `GmshOccBackend` is the first
implementation and can be replaced by fTetWild without changing tasks or scene
construction. Mesh size and IPC contact distance are explicit SI settings.

## Initial standards and task

`d_shaft_6mm_v1` specifies a 6 mm D shaft, 0.75 mm flat depth, 0.20 mm
diametral clearance, 10 mm engagement, 2 mm/s insertion speed, 35 N insertion
force limit, and 0.45 N·m load limit. `mount_30mm_square_v1` supplies the first
support fixture. `RotaryTransmissionSpec` defines `rotary_transmission_v1`
without imposing any internal ratio or candidate motion.

The initial PLA profile is deliberately conservative and isotropic: 1240
kg/m³, 2.3 GPa Young's modulus, 0.36 Poisson ratio, damping 0.02. It is a
baseline to replace with coupon-calibrated profiles; the material interface is
kept separate so slicer-derived orthotropic fields can follow.

## Commands

```bash
uv sync
uv run pytest -q
uv run pytest -q -m integration
uv run python scripts/view_vtk.py build/mechanical_eval/vtk
```

## Current boundary

This slice validates schemas, exact STEP volume separation through OCC,
tetrahedralization, derived properties, STARK deformable-body construction,
physical connector insertion rules, contact-only torque capacity, and reward
integrity. It does not yet implement the complete animated insertion/settle/
brake scene or extract contact reactions/strain energy from STARK. The existing
bindings expose per-node forces and kinematics but not a direct contact-reaction
ledger; that observation hook is the next binding extension before benchmark
rewards are considered authoritative.
