# Trusted evaluator code audit

Status: active pre-lock engineering audit. No physical PASS exists yet.

## Corrected high-severity findings

1. **Python stopped at STARK's first recoverable hardening event.** The harness now retries the same physical timestep while IPC stiffness increases and records retry counts.
2. **Failed Newton iterates leaked into retries.** SymX now snapshots the pre-solve degrees of freedom for every solve and restores them after every unsuccessful result.
3. **Invalid converged contact states did not harden IPC.** The converged-state validity failure now invokes the same hardening callback before rollback/retry.
4. **The Python binding discarded native continue/stop status.** `Simulation.run_one_time_step()` now returns STARK's boolean status.
5. **Contract stiffness did not reach STARK.** IPC minimum/maximum stiffness, contact distance, friction regularization, Newton/CG tolerances, thread count, material, rotor inertias, connector geometry, insertion speed, and insertion-force limit are now contract-wired.
6. **The old 0.1 m/s friction smoothing default was undocumented and too broad for this mechanism.** Contract 1.12.0 explicitly uses 0.0001 m/s. This is pre-lock and must pass sensitivity checks.
7. **Connector slip was wrapped modulo 2π.** Future observations accumulate shaft and candidate rotations and record maximum unwrapped slip over the run.
8. **Integrity was sampled only at the final state.** Future observations record maximum strain, wobble, escape, and slip plus minimum deformation Jacobian over every accepted timestep.
9. **Brake command was mislabeled transmitted torque.** Future observations infer contact torque from evaluator-connector torque balance, including measured angular acceleration and reflected inertia.
10. **Insertion could pass without engagement.** The insertion phase now requires the gap-plus-10-mm travel time, samples force every step, calculates physical overlap, and distinguishes physical FAIL from numerical inconclusiveness.
11. **Finalization trusted one repeat too heavily.** It now enforces CAD/settings hashes, complete three-second duration, exact step count, distinct run paths, policy audit, torque cap, insertion engagement, repeatability of ratio/omega/load, and worst-case values across both repeats.
12. **Run directories were overwritable.** New runs require an empty directory and save their own CAD hash, contract hash, and canonical contract snapshot before simulation.
13. **Evaluator sleeve mesh was inside-out.** Evaluator contact meshes are now made outward-oriented and regression-tested as closed, consistently wound positive-volume surfaces.
14. **Tet meshes were not rejected for inversion/degeneracy/non-manifold boundaries.** Meshing now validates positive signed tet volume and a closed two-manifold contact boundary.
15. **Scoring thresholds were duplicated in code.** Ratio, load, slip, wobble, escape, strain, and minimum-Jacobian gates now come from the frozen contract via the scoring specification.
16. **Newton's 1 mm default step tolerance was 20× the IPC distance.** Contract 1.13.0 scales it to 2.5 μm (5% of the contact distance) so a solve cannot accept a correction that overwhelms the barrier scale.
17. **Most nonlinear-solver controls were inherited defaults.** Contract 1.14.0 explicitly records and wires Newton iteration limits, relative/absolute residual tests, Armijo controls, invalid-state retries, Hessian projection, CG limits/tolerances/indefiniteness policy, and bailout residual.
18. **Evaluator through-pins intersected the output gear.** Candidate 028's unchanged CAD proved the 1.14 fixture invalid; 1.15 splits each pin into upper/lower 4 mm studs with a 0.1 mm axial gap from the gear faces.
19. **Every immutable run recompiled identical SymX kernels.** Evidence output remains append-only, while generated kernels now use a shared reproducible cache outside run artifacts.
20. **Broad bearing plates added unnecessary FEM cost.** Candidate 030 replaces slabs with connected printable bearing rings, mount bosses, and ribs, reducing the full assembly from about 49,000 to 32,000 FEM nodes without removing a physical interface.
21. **Evidence accepted implicit non-finite JSON numbers and incomplete insertion runs.** Finalization now recursively rejects non-finite values, requires complete insertion travel/steps, requires four separately observed candidate bodies, and uses worst-repeat work values.
22. **Native block-PCG is poorly conditioned under sustained D-flat/FEM contact.** The audit added and measured incomplete-Cholesky PCG and ILUT BiCGSTAB alternatives. ILUT produced non-descent directions and is rejected; incomplete Cholesky reduced some iterations but not sustained wall time. Fixed-step and initial-barrier sensitivity runs are retained as numerical diagnostics.

## Verified geometry facts

- Candidate 024 imports as two separate STEP solid volumes.
- The 0.8/1.2 mm mesh contains 5,556 + 18,567 vertices and 21,249 + 83,192 tetrahedra.
- All 104,441 tetrahedra have positive orientation; none are zero-volume.
- Both extracted contact boundaries are closed two-manifolds.
- Volumes are 2,500.914 mm³ and 10,094.833 mm³.
- Evaluator D-shaft and corrected sleeve meshes are watertight, consistently wound, positive-volume meshes with positive-definite inertia.

## Open blockers before final contract lock

1. Candidate 024 is only a two-gear diagnostic assembly. It has no manifest-driven component roles and does not implement the declared 30 mm square mount interface. It cannot mint final reward.
2. The next printable candidate must add candidate-owned physical support/housing geometry and a real evaluator mount fixture; no hidden gear support is permitted.
3. The current `policy_audit()` returns constant assertions rather than an independently derived operation/scene audit. It must be replaced with an auditable construction ledger plus static/runtime enforcement before final evidence.
4. Insertion engagement currently uses axial bounding-box overlap against rest geometry. This does not prove that either D connector physically entered the mating feature; engagement must be measured from connector/candidate geometry and contact state.
5. The attempt ledger must be extended from Candidate 014 through all later diagnostics and include the evaluator-defect reclassification.
6. A focused dynamic contact-only torque-transfer regression must exercise hardening rollback in the native solver.
7. Friction regularization, timestep, initial IPC stiffness, thread performance, surface resolution, and torque-balance observation require final sensitivity/convergence runs. Contract 1.23.0 is a pre-lock 0.5 ms midpoint performance test, not yet a frozen production setting.
8. Authoritative contact-pair/reaction diagnostics are still absent; torque balance is valid for the evaluator connector but direct pair-level hooks would improve debugging.
9. No final PASS, deterministic repeat pair, completed insertion run, final evidence, or success video exists yet.
