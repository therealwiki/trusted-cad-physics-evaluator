# Replaced prototype

The former hinge-constrained rigid gearbox environment has been removed. It
could not provide authoritative mechanical evaluation because evaluator-created
hinges constrained submitted parts and its reward accepted a declared motion
trace. The replacement is documented in `MECHANICAL_EVALUATOR.md` and lives in
the general `mechanical_eval` package.

The VTK viewer remains available:

```bash
uv run python scripts/view_vtk.py build/mechanical_eval/vtk
```

<!-- Historical instructions intentionally removed: they invoked invalid benchmark physics. -->
