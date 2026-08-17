#!/usr/bin/env python3
from __future__ import annotations
import argparse
import re
from collections import defaultdict
from pathlib import Path
import pyvista as pv


def main() -> None:
    parser = argparse.ArgumentParser(description="Play STARK VTK output immediately")
    parser.add_argument("path", nargs="?", type=Path, default=Path("build/gearbox/vtk"))
    args = parser.parse_args()
    files = sorted(args.path.rglob("*.vtk")) if args.path.is_dir() else [args.path]
    if not files:
        raise SystemExit(f"No VTK files found under {args.path}")
    frames = defaultdict(dict)
    for path in files:
        match = re.match(r"(.+)_([0-9]+)$", path.stem)
        if not match:
            continue
        label, frame = match.group(1), int(match.group(2))
        frames[frame][label] = path
    if not frames:
        raise SystemExit("VTK filenames must end in _<frame>.vtk")

    plotter = pv.Plotter()
    actors = {}
    first_frame = min(frames)
    for label, path in frames[first_frame].items():
        actors[label] = plotter.add_mesh(pv.read(path), name=label, show_edges=True)
    plotter.show(auto_close=False, interactive_update=True)
    for frame in sorted(frames):
        for label, path in frames[frame].items():
            if label in actors:
                actors[label].mapper.dataset = pv.read(path)
        plotter.update()
    plotter.show()


if __name__ == "__main__":
    main()
