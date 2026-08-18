from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import gmsh


def _trapezoid_tooth(root_r: float, outer_r: float, thickness: float,
                     center_angle: float, angular_width: float) -> int:
    z0 = -thickness / 2
    half_root = angular_width / 2
    half_tip = half_root * 0.32
    polar = ((root_r - 0.10, center_angle - half_root),
             (outer_r, center_angle - half_tip),
             (outer_r, center_angle + half_tip),
             (root_r - 0.10, center_angle + half_root))
    points = [gmsh.model.occ.addPoint(r * math.cos(a), r * math.sin(a), z0) for r, a in polar]
    lines = [gmsh.model.occ.addLine(points[i], points[(i + 1) % 4]) for i in range(4)]
    loop = gmsh.model.occ.addCurveLoop(lines)
    face = gmsh.model.occ.addPlaneSurface([loop])
    return gmsh.model.occ.extrude([(2, face)], 0, 0, thickness)[1][1]


def _involute_tooth(pitch_r: float, teeth: int, thickness: float,
                    center_angle: float, pressure_angle: float = math.radians(20.0),
                    addendum_factor: float = 1.0) -> int:
    """Create a standard full-depth involute tooth as an extruded planar polygon."""
    module = 2.0 * pitch_r / teeth
    root_r = pitch_r - 1.25 * module
    outer_r = pitch_r + addendum_factor * module
    base_r = pitch_r * math.cos(pressure_angle)
    t_pitch = math.sqrt((pitch_r / base_r) ** 2 - 1.0)
    pitch_involute = t_pitch - math.atan(t_pitch)
    half_tooth = math.pi / (2.0 * teeth)
    flank_rotation = half_tooth + pitch_involute
    t_outer = math.sqrt((outer_r / base_r) ** 2 - 1.0)

    left = []
    right = []
    for i in range(5):
        t = t_outer * i / 4.0
        r = base_r * math.sqrt(1.0 + t * t)
        involute = t - math.atan(t)
        left.append((r, center_angle + flank_rotation - involute))
        right.append((r, center_angle - flank_rotation + involute))
    # Root points connect through the gear's root cylinder during the OCC union.
    polar = [(root_r - 0.10, right[0][1]), *right,
             *reversed(left), (root_r - 0.10, left[0][1])]
    z0 = -thickness / 2
    points = [gmsh.model.occ.addPoint(r * math.cos(a), r * math.sin(a), z0) for r, a in polar]
    lines = [gmsh.model.occ.addLine(points[i], points[(i + 1) % len(points)]) for i in range(len(points))]
    loop = gmsh.model.occ.addCurveLoop(lines)
    face = gmsh.model.occ.addPlaneSurface([loop])
    return gmsh.model.occ.extrude([(2, face)], 0, 0, thickness)[1][1]


def _gear(cx: float, teeth: int, pitch_r: float, thickness: float, bore: bool,
          phase: float = 0.0, tooth_fraction: float = 0.52, profile: str = "rectangular",
          dogbone: bool = False, addendum_factor: float = 1.0,
          pressure_angle_deg: float = 20.0) -> int:
    """Create one conservative spur gear/journal solid in millimetres."""
    module = 2.0 * pitch_r / teeth
    root_r = pitch_r - 1.25 * module
    outer_r = pitch_r + addendum_factor * module
    z0 = -thickness / 2
    parts = [gmsh.model.occ.addCylinder(cx, 0, z0, 0, 0, thickness, root_r)]
    for i in range(teeth):
        angle = 2 * math.pi * i / teeth + phase
        if profile == "involute":
            tooth = _involute_tooth(pitch_r, teeth, thickness, angle,
                                    pressure_angle=math.radians(pressure_angle_deg),
                                    addendum_factor=addendum_factor)
        elif profile == "trapezoid":
            tooth = _trapezoid_tooth(root_r, outer_r, thickness, angle,
                                     tooth_fraction * 2 * math.pi / teeth)
        else:
            tangential = tooth_fraction * (2 * math.pi * pitch_r / teeth)
            tooth = gmsh.model.occ.addBox(root_r - 0.35, -tangential / 2, z0, outer_r - root_r + 0.7,
                                          tangential, thickness)
            gmsh.model.occ.rotate([(3, tooth)], 0, 0, 0, 0, 0, 1, angle)
        gmsh.model.occ.translate([(3, tooth)], cx, 0, 0)
        parts.append(tooth)
    fused, _ = gmsh.model.occ.fuse([(3, parts[0])], [(3, p) for p in parts[1:]], removeObject=True,
                                    removeTool=True)
    gear = fused[0][1]
    # Integral journals are candidate geometry and receive support only by contact.
    journals = [
        gmsh.model.occ.addCylinder(cx, 0, -thickness / 2 - 4, 0, 0, 4, 4.0),
        gmsh.model.occ.addCylinder(cx, 0, thickness / 2, 0, 0, 4, 4.0),
    ]
    fused, _ = gmsh.model.occ.fuse([(3, gear)], [(3, j) for j in journals], removeObject=True, removeTool=True)
    gear = fused[0][1]
    if bore:
        # 6 mm D mating feature with the locked 0.20 mm diametral clearance.
        disk = gmsh.model.occ.addCylinder(cx, 0, -thickness / 2 - 5, 0, 0, thickness + 10, 3.1)
        clip = gmsh.model.occ.addBox(cx + 2.35, -4, -thickness / 2 - 5, 3, 8, thickness + 10)
        d_hole, _ = gmsh.model.occ.cut([(3, disk)], [(3, clip)], removeObject=True, removeTool=True)
        # Printable dog-bone reliefs prevent the sharp shaft/flat corners from
        # becoming contact singularities while leaving the central flat as the
        # physical torque-carrying interface.
        tools = d_hole
        if dogbone:
            relief_y = math.sqrt(3.1 ** 2 - 2.35 ** 2)
            reliefs = [gmsh.model.occ.addCylinder(cx + 2.55, sign * relief_y, -thickness / 2 - 5,
                                                 0, 0, thickness + 10, 0.60) for sign in (-1, 1)]
            tools, _ = gmsh.model.occ.fuse(d_hole, [(3, tag) for tag in reliefs],
                                            removeObject=True, removeTool=True)
        cut, _ = gmsh.model.occ.cut([(3, gear)], tools, removeObject=True, removeTool=True)
        gear = cut[0][1]
    return gear


def _housing_plate(z_center: float, gear_centers_x: tuple[float, float]) -> int:
    """Printable bearing/mount plate; all support is supplied through its contact geometry."""
    thickness = 4.0
    z0 = z_center - thickness / 2
    parts = [gmsh.model.occ.addCylinder(cx, 0.0, z0, 0, 0, thickness, 7.0)
             for cx in gear_centers_x]
    parts.extend(gmsh.model.occ.addCylinder(x, y, z0, 0, 0, thickness, 4.0)
                 for x in (-15.0, 15.0) for y in (-15.0, 15.0))
    # One central drive-train rib, two mount rails, and two cross-ribs form a
    # single printable support body without a broad, numerically expensive slab.
    parts.append(gmsh.model.occ.addBox(-15.0, -2.0, z0, 36.5, 4.0, thickness))
    parts.extend(gmsh.model.occ.addBox(-17.0, y - 2.0, z0, 34.0, 4.0, thickness)
                 for y in (-15.0, 15.0))
    parts.extend(gmsh.model.occ.addBox(x - 2.0, -15.0, z0, 4.0, 30.0, thickness)
                 for x in (-15.0, 15.0))
    fused, _ = gmsh.model.occ.fuse([(3, parts[0])], [(3, part) for part in parts[1:]],
                                   removeObject=True, removeTool=True)
    plate = fused[0][1]
    tools = [gmsh.model.occ.addCylinder(cx, 0.0, z_center - 3.0, 0, 0, 6.0, 4.3)
             for cx in gear_centers_x]
    tools.extend(gmsh.model.occ.addCylinder(x, y, z_center - 3.0, 0, 0, 6.0, 1.75)
                 for x in (-15.0, 15.0) for y in (-15.0, 15.0))
    cut, _ = gmsh.model.occ.cut([(3, plate)], [(3, tool) for tool in tools],
                                removeObject=True, removeTool=True)
    return cut[0][1]


def generate(path: Path, tooth_fraction: float, center_distance: float, profile: str, dogbone: bool,
             addendum_factor: float, pressure_angle_deg: float, housing: bool = False,
             input_teeth: int = 12, output_teeth: int = 24) -> None:
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 1)
        gmsh.model.add("gearbox_candidate_001")
        if output_teeth != 2 * input_teeth:
            raise ValueError("rotary_transmission_v1 requires an exact 2:1 tooth-count ratio")
        # The pitch radii and center distance remain contract-facing geometry;
        # tooth counts may change during pre-lock design optimization.
        input_gear = _gear(-10.0, input_teeth, 10.0, 8.0, True, tooth_fraction=tooth_fraction, profile=profile,
                           dogbone=dogbone, addendum_factor=addendum_factor,
                           pressure_angle_deg=pressure_angle_deg)
        output_gear = _gear(-10.0 + center_distance, output_teeth, 20.0, 8.0, True,
                            math.pi / output_teeth,
                            tooth_fraction, profile, dogbone, addendum_factor, pressure_angle_deg)
        gmsh.model.setEntityName(3, input_gear, f"input_gear_{input_teeth}t")
        gmsh.model.setEntityName(3, output_gear, f"output_gear_{output_teeth}t")
        component_roles = [
            {"role": "input_gear", "expected_centroid_mm": [-10.0, 0.0, 0.0]},
            {"role": "output_gear", "expected_centroid_mm": [-10.0 + center_distance, 0.0, 0.0]},
        ]
        if housing:
            lower = _housing_plate(-6.1, (-10.0, -10.0 + center_distance))
            upper = _housing_plate(6.1, (-10.0, -10.0 + center_distance))
            gmsh.model.setEntityName(3, lower, "candidate_lower_bearing_plate")
            gmsh.model.setEntityName(3, upper, "candidate_upper_bearing_plate")
            component_roles.extend([
                {"role": "lower_bearing_plate", "expected_centroid_mm": [4.0, 0.0, -6.1]},
                {"role": "upper_bearing_plate", "expected_centroid_mm": [4.0, 0.0, 6.1]},
            ])
        gmsh.model.occ.synchronize()
        path.parent.mkdir(parents=True, exist_ok=True)
        gmsh.write(str(path))
        manifest = {
            "schema_version": "1.0",
            "step_path": path.name,
            "components": component_roles,
            "ports": {
                "input": {"origin_mm": [-10.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0]},
                "output": {"origin_mm": [-10.0 + center_distance, 0.0, 0.0], "axis": [0.0, 0.0, 1.0]},
                "mount": {"origin_mm": [0.0, 0.0, 0.0], "axis": [0.0, 0.0, 1.0]},
            },
        }
        path.with_suffix(".manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    finally:
        gmsh.finalize()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("output", type=Path)
    parser.add_argument("--tooth-fraction", type=float, default=0.52)
    parser.add_argument("--center-distance-mm", type=float, default=30.0)
    parser.add_argument("--profile", choices=("rectangular", "trapezoid", "involute"), default="rectangular")
    parser.add_argument("--dogbone", action="store_true")
    parser.add_argument("--addendum-factor", type=float, default=1.0)
    parser.add_argument("--pressure-angle-deg", type=float, default=20.0)
    parser.add_argument("--input-teeth", type=int, default=12)
    parser.add_argument("--output-teeth", type=int, default=24)
    parser.add_argument("--housing", action="store_true")
    args = parser.parse_args()
    generate(args.output.resolve(), args.tooth_fraction, args.center_distance_mm, args.profile, args.dogbone,
             args.addendum_factor, args.pressure_angle_deg, args.housing,
             args.input_teeth, args.output_teeth)
