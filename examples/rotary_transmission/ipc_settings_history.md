# IPC and timestep history — Candidates 001–030

This is an execution audit, not a retrospective statement of intended settings. Values are taken from native STARK logs, observation contract hashes, preserved source history, and the immutable attempt ledger. `effective` means the value that reached STARK. Where the surviving artifacts cannot prove a value, it is explicitly marked.

## Shared solver configuration

- Integrator: STARK implicit time integration with fixed timestep; adaptive timestep disabled.
- Contact: triangle/edge IPC barrier contact with continuous collision checks and friction enabled.
- Material during the recorded optimization: stable Neo-Hookean tetrahedral FEM; the current locked profile is effective isotropic printed PLA, density 1200 kg/m³, Young's modulus 2.5 GPa, Poisson ratio 0.35.
- Newton absolute residual tolerance: `1e-6` from contract 1.1.0 onward.
- Newton step tolerance: `1e-3` from contract 1.1.0 onward.
- BDPCG absolute/relative tolerances: `1e-12` / `1e-4` from contract 1.1.0 onward.
- Maximum IPC stiffness: `1e12` for the later controlled runs.
- Friction coefficient: 0.35.
- IPC friction stick/slip smoothing threshold: STARK default 0.1 m/s; this was not previously made contract-explicit and must be included in the final lock.

## Candidate-by-candidate execution settings

| Candidate | Contract/run family | Fixed dt | Effective pair IPC distance | Effective surface / volume target | Effective initial IPC stiffness | Load protocol | Evidence and audit note |
|---:|---|---:|---:|---:|---:|---|---|
| 001 | foundation bring-up | 1.00 ms | 0.10 mm | 1.2 / 1.2 mm | 1e6 default | no valid actuation | Initial overlap; contact thickness was mistakenly applied in full to both objects. |
| 002 | foundation bring-up | 1.00 ms | 0.10 mm | 1.2 / 1.2 mm | 1e6 default | no valid actuation | Initial overlap. |
| 003 | foundation bring-up | 1.00 ms | 0.10 mm | 1.2 / 1.2 mm | 1e6 default | no valid actuation | Initial overlap. |
| 004 | foundation diagnostics | 1.00 ms | 0.10 mm initially; corrected to 0.05 mm in later sub-run | 1.2 / 1.2 mm | 1e6 default | several 0.02 s diagnostics | Multiple preserved sub-runs; exposed doubled pair-distance adapter bug. |
| 005 | contracts 1.0.x–1.1.0 diagnostics | 1.00 ms | 0.05 mm after pair-distance fix | 1.2 / 1.2 mm | 1e6 default | up to 0.25 s | Multiple control diagnostics; tighter solver tolerances introduced. |
| 006 | 1.1.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e6, log-proven | 0.35 s requested | First hardening at 0.130 s. Old harness stopped instead of retrying. |
| 007 | 1.1.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e6 | no actuation | Rejected at initial collision. |
| 008 | 1.1.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e6 | no actuation | Rejected at initial collision. |
| 009 | 1.1.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e6 | no actuation | Rejected at initial collision. |
| 010 | 1.1.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e6 | no actuation | Rejected at initial contact-distance check. |
| 011 | 1.1.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e6, log-proven | 0.35 s requested | First hardening at 0.125 s; old harness stopped instead of retrying. |
| 012 | 1.2.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e7, log-proven | 0.35 s requested | First hardening at 0.130 s; old harness stopped instead of retrying. |
| 013 | 1.3.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e7, log-proven | staged speed; brake after take-up | First hardening at 0.201 s; old harness stopped instead of retrying. |
| 014 | 1.4.0 | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e8, log-proven | 0.4 N·m/s brake ramp | 0.300 s gate completed; loaded repeat first hardened at 0.543 s and 0.0172 N·m. |
| 015 | 1.5.0 diagnostic | 1.00 ms | 0.05 mm | 1.2 / 1.2 mm | 1e9 intended; not independently log-proven | fast brake ramp | Interrupted at 0.038 s to run timestep study; not a scored attempt. |
| 016 | 1.6.0 | 0.50 ms | 0.05 mm | 1.2 / 1.2 mm | 1e8, log-proven | 0.4 N·m/s brake ramp | First hardening at 0.544 s and about 0.0176 N·m. |
| 017 | 1.6.0 | 0.50 ms | 0.05 mm | 1.2 / 1.2 mm | 1e8 | no actuation | D-flat relief revision rejected by initial collision. |
| 018 | 1.6.0 | 0.50 ms | 0.05 mm | 1.2 / 1.2 mm | 1e8, log-proven | staged speed | First hardening at 0.2355 s before brake. |
| 019 | 1.6.0 | 0.50 ms | 0.05 mm | 1.2 / 1.2 mm | 1e8, log-proven | 0.4 N·m/s brake ramp | First hardening at 0.549 s and 0.01884 N·m. |
| 020 | 1.6.0 | 0.50 ms | 0.05 mm | 0.8 / 1.2 mm | 1e8, log-proven | 0.4 N·m/s brake ramp | Surface-mesh plumbing fixed; first hardening at 0.5555 s and 0.01405 N·m. |
| 021 | 1.7.0 | 0.25 ms | 0.05 mm | 0.8 / 1.2 mm | 1e8, log-proven | 0.4 N·m/s brake ramp | First hardening at 0.55675 s. |
| 022 | 1.8.0 | 0.25 ms | 0.05 mm | 0.8 / 1.2 mm | 1e8, log-proven | 0.4 N·m/s brake ramp | Separate evaluator rotor inertias; first hardening at 0.5545 s and 0.01798 N·m. |
| 023 | 1.8.0 | 0.25 ms | 0.05 mm | 0.8 / 1.2 mm | 1e8, log-proven | 0.4 N·m/s brake ramp | First hardening at 0.53425 s and 0.01370 N·m. |
| 024 | 1.9.0 | 0.25 ms | 0.05 mm | 0.8 / 1.2 mm | 1e8, log-proven | 0.08 N·m/s brake ramp | First hardening at 0.76375 s and 0.02110 N·m. |
| 025 | 1.9.0 | 0.25 ms | 0.05 mm | 0.8 / 1.2 mm | 1e8, log-proven | 0.08 N·m/s brake ramp | First hardening at 0.74975 s and 0.01998 N·m. |
| 026 | 1.9.0 | 0.25 ms | 0.05 mm | 0.8 / 1.2 mm | 1e8, log-proven | 0.08 N·m/s brake ramp | First hardening at 0.76125 s and 0.02090 N·m. |

## Why the prior stops were not mechanical breaks

STARK's native `run_one_step()` treats `InvalidConvergedState` and `TooManyInvalidIntermediateIterations` as recoverable: it hardens the IPC barrier, does not advance physical time, and returns a continue signal so the caller retries the same timestep. The old Python harness interpreted one non-advancing call as terminal. Therefore Candidates 006–026 that ended at their first hardening event are evaluator-integration failures and must not be presented as broken parts.

Contract 1.10.0 fixed the settings plumbing and retry protocol, then its expensive 0.25 ms / 2e8 diagnostic was deliberately interrupted before the old failure point when the performance audit showed that those conservative settings predated the root-cause fix. Contract 1.11.0 restores a 1.00 ms step and 1e8 initial barrier while retaining the same physical load history. It is the fast baseline for a controlled 1.00/0.50/0.25 ms convergence study. Final scoring requires a newly frozen contract and clean repeat runs.

## Post-retry architecture and conditioning audit

| Candidate | Contract | dt | Initial IPC k | Linear solve | Outcome |
|---:|---:|---:|---:|---|---|
| 024 | 1.12–1.13 | 1.00 ms | 1e8 | BDPCG | Native rollback and 2.5 µm Newton-step tolerance smoke tests completed 50 ms; diagnostic two-solid assembly only. |
| 027 | pre-1.15 | 1.00 ms | 1e8 | BDPCG | First four-solid CAD with candidate bearing plates; mount-hole radial gap equaled the IPC distance, so it was superseded before actuation. |
| 028 | 1.14 | 1.00 ms | 1e8 | BDPCG | Rejected because evaluator through-pins intersected the output gear; evaluator defect, not candidate failure. |
| 028 | 1.15 | 1.00 ms | 1e8 | BDPCG | Unchanged STEP passed 1 ms and 50 ms gates after split fixture studs; no torque-transfer claim. |
| 029 | 1.15–1.17 | 1.00 ms | 1e8 | BDPCG / ICPCG | Trimmed plates reduced nodes 22%; sustained take-up exposed thousands of Krylov iterations. ICPCG and 1% inexact-Newton trials were retained as interrupted numerical diagnostics. |
| 030 | 1.18 | 1.00 ms | 1e8 | ICPCG | Printable ribbed bearing frames reduced the assembly to about 32k nodes, but contact conditioning remained dominant. |
| 030 | 1.19 | 1.00 ms | 1e7 | BDPCG | Lower initial barrier improved early contact cost but sustained take-up again became expensive. |
| 030 | 1.20 | 0.25 ms | 1e7 | BDPCG | Fewer iterations per step but fourfold step count was slower per simulated millisecond; rejected for production. |
| 030 | 1.21 | 1.00 ms | 1e7 | ILUT BiCGSTAB | Rejected immediately: non-descent directions triggered repeated Hessian-projection retries. |
| 030 | 1.22 | 1.00 ms | 1e7 | ICPCG | Better early coupling but sustained take-up again reached roughly 10 s/step; rejected. |
| 030 | 1.23 | 0.50 ms | 1e7 | BDPCG | Completed all 220 steps through 0.110 s with zero hardening retries, but required about 15 minutes and ended before brake onset. Stable early gate; rejected for production speed and not eligible for final reward. |

All interrupted rows are `INCONCLUSIVE_NUMERICS` performance studies. None is evidence of a broken printed part or a physical PASS.
