# Trusted rotary transmission example

This directory owns the complete `rotary_transmission_v1` benchmark and client demo. It contains the evolving pre-lock contract, candidate STEP assemblies, manifests, meshes, immutable run evidence, rendering sources, and final media. Generic evaluator and STARK binding improvements remain in the repository libraries.

The effective IPC settings and timestep used for Candidates 001–030 are reconstructed in [`ipc_settings_history.md`](ipc_settings_history.md). That audit distinguishes intended contract values from values proven to have reached STARK. Contract 1.23.0 is an active pre-lock conditioning study; a canonical scoring hash will be declared only after the corrected retry protocol, dynamic-contact regression, and convergence study pass and the physical contract is frozen for clean repeats.

Version 1.1.0 adopts the user-directed effective isotropic printed-PLA profile: 1,200 kg/m³, 2.5 GPa, Poisson ratio 0.35, and a 2% elastic-strain acceptance ceiling. It is a recoverable stable Neo-Hookean model, not a yielding, fracture, fatigue, thermal, or layer-separation model. Exploratory runs retain their original settings hashes in the attempt ledger.

Version 1.4.0 introduced the staged speed/brake schedule and a 1e8 minimum IPC barrier stiffness during pre-lock numerical bring-up. Later validation exposed settings-plumbing and contact-hardening retry defects, so it is retained as historical evidence rather than misrepresented as the final scoring lock. No material property or physical acceptance threshold was weakened.

Calibration should use the production print recipe: weigh a dimensional coupon, measure tensile or flexural modulus, fit damping from torsional decay or relaxation, and test D-shaft insertion/slip for friction and clearance.

No analytic capacity estimate, declared trace, or scripted rotation is accepted as physical success.

## Layout

- `contracts/`: versioned evaluator-owned test contract
- `cad/`: every substantive STEP candidate and its adjacent immutable component/port manifest
- `meshes/`: independently generated tetrahedral meshes
- `runs/`: logs, observations, evidence, and attempt visuals
- `render/`: Blender/PyVista/FFmpeg project sources and frames
- `outputs/`: final client-facing deliverables

Candidate 031 is the current compact four-body architecture: separate input/output FEM gears and separate upper/lower FEM bearing frames. Evaluator-owned split mount studs contact the candidate frames; no evaluator bearing directly supports a gear. Its 8:16 involute pair preserves the 2:1 ratio while using module-scaled root and addendum geometry.

## Current reproducible commands

```bash
uv sync --reinstall-package pystark
uv run pytest -q

uv run python examples/rotary_transmission/generate_gearbox_step.py \
  examples/rotary_transmission/cad/candidate_031.step \
  --profile involute --center-distance-mm 31.5 --pressure-angle-deg 20 --housing \
  --input-teeth 8 --output-teeth 16

uv run python examples/rotary_transmission/run_harness.py \
  examples/rotary_transmission/cad/candidate_031.step \
  examples/rotary_transmission/runs/candidate_031_contract_1_24_0_full_a \
  --duration 3 --actuation
```

The contract is deliberately marked pre-lock. Do not run `finalize_evidence.py` or label a run PASS until the contract is frozen and a complete insertion run plus two distinct full-duration repeats exist.
