# Trusted rotary transmission example

This directory owns the complete `rotary_transmission_v1` benchmark and client demo. It contains the evolving pre-lock contract, candidate STEP assemblies, manifests, meshes, immutable run evidence, rendering sources, and final media. Generic evaluator and STARK binding improvements remain in the repository libraries.

The effective IPC settings and timestep used for Candidates 001–026 are reconstructed in [`ipc_settings_history.md`](ipc_settings_history.md). That audit distinguishes intended contract values from values proven to have reached STARK. Contract 1.10.0 remains a numerical-validation amendment; a new canonical hash will be declared only after the corrected retry protocol passes validation and the scoring contract is frozen for clean repeats.

Version 1.1.0 adopts the user-directed effective isotropic printed-PLA profile: 1,200 kg/m³, 2.5 GPa, Poisson ratio 0.35, and a 2% elastic-strain acceptance ceiling. It is a recoverable stable Neo-Hookean model, not a yielding, fracture, fatigue, thermal, or layer-separation model. Exploratory runs retain their original settings hashes in the attempt ledger.

Version 1.4.0 introduced the staged speed/brake schedule and a 1e8 minimum IPC barrier stiffness during pre-lock numerical bring-up. Later validation exposed settings-plumbing and contact-hardening retry defects, so it is retained as historical evidence rather than misrepresented as the final scoring lock. No material property or physical acceptance threshold was weakened.

Calibration should use the production print recipe: weigh a dimensional coupon, measure tensile or flexural modulus, fit damping from torsional decay or relaxation, and test D-shaft insertion/slip for friction and clearance.

No analytic capacity estimate, declared trace, or scripted rotation is accepted as physical success.

## Layout

- `contracts/`: versioned evaluator-owned test contract
- `cad/`: every substantive STEP candidate
- `manifests/`: candidate component and port metadata
- `meshes/`: independently generated tetrahedral meshes
- `runs/`: logs, observations, evidence, and attempt visuals
- `render/`: Blender/PyVista/FFmpeg project sources and frames
- `outputs/`: final client-facing deliverables
