# Rotary transmission evaluator — 105 second storyboard

All technical geometry comes from the preserved STEP/tet artifacts. All moving candidate geometry comes from `recorded_frames.npz`; editorial time-remapping does not alter measured states.

| Time | Picture | Evidence / copy |
|---:|---|---|
| 0–8 s | Empty dark test cell; evaluator D shafts, sleeves, envelope and datums only | **A test cell with no opinion about the design.** |
| 8–18 s | Contract dimensions animate around the rig | 6 mm D, 10 mm engagement, 2 mm/s insertion, 35 N insertion cap, 0.25 N·m motor cap, 0.08 N·m brake, −2:1 target. **Metadata locates equipment. Geometry carries the load.** |
| 18–25 s | Pipeline schematic | STEP → exact solids → separate tet FEM bodies → insert → settle → actuate/load → measured evidence. No candidate joints, attachments, prescribed motion, or direct force. |
| 25–31 s | Agent/CAD generator and first two separate solids | Effective isotropic printed-PLA: 1200 kg/m³, 2.5 GPa, ν 0.35; stable Neo-Hookean recoverable elasticity. |
| 31–47 s | Attempts 001–005, each identified; rejected geometry and native reason | Initial overlap ×3; corrected clearance; numerical/control bring-up. No physical reward. |
| 47–63 s | Attempts 006–010 | Tapered-tooth penetration; three preserved defective-involute collision rejects; corrected involute still inside contact clearance. |
| 63–76 s | Attempts 011–013 as actual state captures | Correct direction but hardening at 125 ms; stronger barrier at 130 ms; staged take-up engages output D-flat then hardens at 201 ms. Each labeled `INCONCLUSIVE_NUMERICS`. |
| 76–91 s | Candidate 014 insertion, settle, full actuation and brake load from actual frames | Connector force, shaft/gear omega, torque, ratio, strain and wobble update from recorded observations. |
| 91–99 s | Auditable evidence panel | Classification and actual values; CAD/settings hashes; repeatability; policy audit. Never show PASS unless the final evidence file says PASS. |
| 99–105 s | Exploded/cutaway hero from final geometry | **The evaluator never trusted the design. It tested the geometry.** Simulation-model disclaimer. |

Visual system: evaluator cyan, candidate warm aluminum/amber, active contact magenta, pass green, failure red, inconclusive amber. Text remains inside 10% title-safe margins.
