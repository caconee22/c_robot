from pathlib import Path

import cadquery as cq
from cadquery import exporters, importers


ROOT = Path(__file__).resolve().parents[1]
DXF_PATH = ROOT / "pulley_HTD_5mm_20.DXF"
OUT_DIR = ROOT / "output" / "HTD5M_20T_8mm_15mm"

# Pulley specification (millimetres)
TOOTH_FACE_WIDTH = 16.0  # 0.5 mm side clearance for a 15 mm belt
BORE_DIAMETER = 8.0
FLANGE_DIAMETER = 36.0
FLANGE_THICKNESS = 1.0


def build_pulley() -> cq.Workplane:
    tooth_profile = importers.importDXF(str(DXF_PATH))
    toothed_body = tooth_profile.extrude(TOOTH_FACE_WIDTH / 2.0, both=True)

    left_flange = (
        cq.Workplane("XY", origin=(0, 0, -TOOTH_FACE_WIDTH / 2.0 - FLANGE_THICKNESS))
        .circle(FLANGE_DIAMETER / 2.0)
        .extrude(FLANGE_THICKNESS)
    )
    right_flange = (
        cq.Workplane("XY", origin=(0, 0, TOOTH_FACE_WIDTH / 2.0))
        .circle(FLANGE_DIAMETER / 2.0)
        .extrude(FLANGE_THICKNESS)
    )

    pulley = toothed_body.union(left_flange).union(right_flange)
    return pulley.faces(">Z").workplane().hole(BORE_DIAMETER)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    pulley = build_pulley()

    exporters.export(pulley, str(OUT_DIR / "HTD5M_20T_Bore8_Belt15.step"))
    exporters.export(
        pulley,
        str(OUT_DIR / "HTD5M_20T_Bore8_Belt15.stl"),
        tolerance=0.02,
        angularTolerance=0.05,
    )

    shape = pulley.val()
    box = shape.BoundingBox()
    print(f"Overall size: {box.xlen:.3f} x {box.ylen:.3f} x {box.zlen:.3f} mm")
    print(f"Volume: {shape.Volume():.3f} mm^3")
    print(f"Solids: {len(shape.Solids())}")


if __name__ == "__main__":
    main()
