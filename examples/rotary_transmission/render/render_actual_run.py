from __future__ import annotations

import argparse
from pathlib import Path
import subprocess

import numpy as np
import pyvista as pv

from mechanical_eval.mesh import GmshOccBackend
from mechanical_eval.contract import load_contract


CYAN = "#40d9ff"
AMBER = "#d89b45"
MAGENTA = "#ff4ca6"
BG = "#070b11"


def surface(vertices: np.ndarray, triangles: np.ndarray) -> pv.PolyData:
    faces = np.column_stack((np.full(len(triangles), 3), triangles)).ravel()
    return pv.PolyData(vertices, faces)


def render(step: Path, run_dir: Path, output: Path, width: int, height: int) -> None:
    contract = load_contract(Path(__file__).parents[1] / "contracts" / "rotary_transmission_v1.json",
                             allow_prelock=True)
    meshes = GmshOccBackend().mesh_step(step, size_m=contract["mesh_and_contact"]["volume_size_m"])
    frames = np.load(run_dir / "recorded_frames.npz", allow_pickle=True)["frames"]
    frame_dir = output.parent / f".{output.stem}_frames"
    frame_dir.mkdir(parents=True, exist_ok=True)

    plotter = pv.Plotter(off_screen=True, window_size=(width, height), lighting="three lights")
    plotter.set_background(BG)
    actors = []
    for index, mesh in enumerate(meshes):
        poly = surface(np.asarray(frames[0]["candidate"][index]["points_m"]), mesh.surface_triangles)
        actors.append((poly, plotter.add_mesh(poly, color=AMBER, metallic=0.55, roughness=0.3,
                                              smooth_shading=True, specular=0.7)))
    # Evaluator-owned bearing sleeves and datum/envelope are deliberately cyan.
    centers = [m.vertices_m.mean(0) for m in meshes]
    for center in centers:
        for z in (-0.0061, 0.0061):
            ring = pv.Cylinder(center=(center[0], 0, z), direction=(0, 0, 1), radius=0.0058, height=0.004,
                               resolution=64).triangulate()
            plotter.add_mesh(ring, color=CYAN, opacity=0.28, metallic=0.7, roughness=0.22)
    envelope = pv.Box(bounds=(-0.025, 0.045, -0.027, 0.027, -0.014, 0.014))
    plotter.add_mesh(envelope, style="wireframe", color=CYAN, opacity=0.13, line_width=1)
    plotter.camera_position = [(0.060, -0.090, 0.066), (0.005, 0.0, 0.0), (0, 0, 1)]

    for frame_index, frame in enumerate(frames):
        for body_index, (poly, _) in enumerate(actors):
            poly.points = np.asarray(frame["candidate"][body_index]["points_m"])
        plotter.add_text(f"CONTACT-ONLY FEM  |  t = {float(frame['time_s']):.3f} s", position=(70, height - 90),
                         color="white", font_size=13, name="time_overlay")
        plotter.add_text(f"motor {float(frame['motor_torque_nm']):+.3f} N·m    brake {float(frame['brake_torque_nm']):+.3f} N·m",
                         position=(70, 55), color=MAGENTA, font_size=12, name="load_overlay")
        plotter.screenshot(frame_dir / f"frame_{frame_index:05d}.png")
    plotter.close()
    ffmpeg = "/opt/homebrew/bin/ffmpeg"
    subprocess.run([ffmpeg, "-y", "-framerate", "30", "-i", str(frame_dir / "frame_%05d.png"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "15", str(output)], check=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("step", type=Path)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    render(args.step.resolve(), args.run_dir.resolve(), args.output.resolve(), args.width, args.height)
