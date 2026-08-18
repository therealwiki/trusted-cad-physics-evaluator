from __future__ import annotations

import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).parents[1]
OUT = ROOT / "render" / "cards"
W, H = 3840, 2160
BG = (7, 11, 17)
WHITE = (235, 242, 247)
MUTED = (137, 154, 167)
CYAN = (64, 217, 255)
AMBER = (216, 155, 69)
RED = (255, 91, 91)
GREEN = (85, 225, 145)


def font(size: int, bold: bool = False):
    paths = ["/System/Library/Fonts/SFNS.ttf", "/System/Library/Fonts/Helvetica.ttc"]
    for path in paths:
        try:
            return ImageFont.truetype(path, size=size, index=1 if bold and path.endswith("ttc") else 0)
        except OSError:
            pass
    return ImageFont.load_default()


def card(name: str, eyebrow: str, title: str, lines: list[str], accent=CYAN, footer=""):
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)
    d.rectangle((0, 0, 34, H), fill=accent)
    d.text((340, 250), eyebrow.upper(), font=font(54, True), fill=accent)
    d.text((340, 390), title, font=font(140, True), fill=WHITE)
    y = 730
    for line in lines:
        d.ellipse((350, y + 28, 370, y + 48), fill=accent)
        d.text((420, y), line, font=font(62), fill=WHITE if not line.startswith("↳") else MUTED)
        y += 135
    if footer:
        d.text((340, 1950), footer, font=font(38), fill=MUTED)
    d.line((340, 1850, 3500, 1850), fill=(35, 51, 63), width=3)
    im.save(OUT / f"{name}.png")


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    card("01_empty", "ROTARY TRANSMISSION V1", "An empty test cell.",
         ["Evaluator-owned motor, D shafts, brake and fixture", "No candidate joints. No hidden support.",
          "The submitted geometry must create the causal path."], footer="TRUSTED MECHANICAL CAD EVALUATION")
    card("02_contract", "FROZEN CONTRACT", "Geometry carries the load.",
         ["6.00 mm D interface  •  10 mm engagement  •  0.20 mm diametral clearance",
          "2 mm/s insertion  •  <35 N  •  0.25 N·m motor limit  •  0.08 N·m brake",
          "Target −2.00:1  •  tolerance ±0.15  •  fixed 1 ms steps"], footer="Metadata locates equipment. It cannot transmit torque.")
    card("03_pipeline", "TRUSTED PIPELINE", "Evidence, not declarations.",
         ["STEP → exact CAD → separate solids → tetrahedral FEM",
          "Physical insertion → settle → speed-limited actuation → evaluator brake",
          "Observe ratio, slip, wobble, strain, work, escape and solver status"], footer="Synthetic or hand-authored traces cannot mint reward.")
    card("04_material", "MODEL SCOPE", "Effective isotropic printed-PLA.",
         ["ρ 1200 kg/m³  •  E 2.5 GPa  •  ν 0.35  •  μ 0.35",
          "Stable Neo-Hookean recoverable elasticity  •  strain gate <2%",
          "No yielding, fracture, fatigue, heat or layer-separation model"], accent=AMBER,
         footer="Literature-informed vertical slice; print-recipe calibration remains required.")
    card("05_attempts_a", "ATTEMPTS 001—005", "Reject. Measure. Revise.",
         ["001–003  Initial solid overlap — FAIL", "004  Adapter/contact-distance bring-up — INCONCLUSIVE",
          "005  Control residual prevented authoritative actuation — INCONCLUSIVE"], accent=RED)
    card("06_attempts_b", "ATTEMPTS 006—010", "The CAD generator is testable too.",
         ["006  Tapered teeth: correct direction, then penetration hardening",
          "007–009  Defective involute widened at tip — initial collision",
          "010  Corrected flank, insufficient center clearance — initial collision"], accent=RED)
    card("07_attempts_c", "ATTEMPTS 011—013", "Physical progress. Still no pass.",
         ["011  1.945:1, correct direction; hardening at 125 ms",
          "012  Stronger barrier; hardening at 130 ms",
          "013  Output D-flat engages; hardening at 201 ms"], accent=AMBER,
         footer="Every numerical termination is INCONCLUSIVE_NUMERICS—not a jam.")
    evidence_path = ROOT / "outputs" / "final_evidence.json"
    if evidence_path.exists():
        evidence = json.loads(evidence_path.read_text())
        status = evidence.get("classification", "UNSCORED")
        accent = GREEN if status == "PASS" else AMBER
        values = evidence.get("observations", {})
        card("10_evidence", "AUDITABLE RESULT", status,
             [f"ratio {values.get('ratio', 'pending')}  •  load {values.get('output_load_nm', 'pending')} N·m",
              f"max strain {values.get('max_strain', 'pending')}  •  wobble {values.get('max_wobble_m', 'pending')} m",
              f"solver {values.get('solver_status', 'pending')}",
              f"CAD {evidence.get('cad_sha256', 'pending')[:16]}…  settings {evidence.get('settings_sha256', 'pending')[:16]}…"], accent=accent)
    else:
        card("10_evidence", "AUDITABLE RESULT", "PENDING PHYSICAL GATES",
             ["No PASS is rendered until insertion, brake load and repeatability agree."], accent=AMBER)
    card("11_end", "TRUSTED EVALUATION", "The evaluator never trusted the design.",
         ["It tested the geometry."], accent=CYAN,
         footer="Simulation engineering evaluation under the shown model/settings; not real-world safety certification.")


if __name__ == "__main__":
    main()
