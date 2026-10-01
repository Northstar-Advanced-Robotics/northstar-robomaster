#!/usr/bin/env python3
"""Generate CAD models of the game elements: cube, tube cup and buzz-wire stand.

All dimensions are in millimetres. The cube follows the official drawing
(200 mm cube, 164 mm flat faces, so an 18 mm x 45 deg chamfer on every edge).
Everything else was measured off the reference render using that cube as the
scale (about 0.905 px/mm, camera elevation about 17.5 deg), then rounded.

Usage (from this directory):
    pip install -r requirements.txt
    python generate.py

Writes step/, stl/ and scene.glb next to this script.
"""

import math
from pathlib import Path

from build123d import (
    Axis,
    Circle,
    Color,
    Compound,
    Cylinder,
    FilletPolyline,
    Location,
    Plane,
    Polygon,
    Pos,
    Rectangle,
    RectangleRounded,
    Rot,
    Torus,
    export_gltf,
    export_step,
    export_stl,
    extrude,
    fillet,
    sweep,
)

OUT_DIR = Path(__file__).resolve().parent

# Cube, from the drawing
CUBE_SIZE = 200.0
CUBE_CHAMFER = 18.0  # 200 - 2 * 18 = 164 mm flat face
DECAL_THICKNESS = 0.3  # logos / markers are thin separate bodies on the faces

# Fiducial markers, from the drawing: 30 mm markers on a 150 mm square,
# L-bracket arms 10 mm wide, 80 x 50 mm barcode on the bottom face
MARKER_SQUARE = 150.0
MARKER_SIZE = 30.0
MARKER_ARM = 10.0
BARCODE_W = 80.0
BARCODE_H = 50.0
BARCODE_PATTERN = "1101001011011001010011010110100101101011"  # 2 mm modules, 80 mm wide

# Lightning logo from the render, upper half in face coordinates (mm from the
# face centre, x right, y up). The lower half is the same shape rotated 180 deg.
LOGO_HALF = [
    (-47.5, 2.0),
    (-20.5, 2.0),
    (26.0, 42.0),
    (6.0, 2.0),
    (46.5, 2.0),
    (47.5, 21.0),
    (38.0, 21.5),
    (60.0, 67.0),
    (25.6, 67.0),
]

# Cup and tubes, scaled from the render
CUP_OD = 420.0
CUP_HEIGHT = 510.0
CUP_WALL = 5.0
CUP_FLOOR = 10.0
TUBE_OD = 50.0
TUBE_ID = 40.0
TUBE_LENGTH = 1000.0

# Buzz-wire stand, scaled from the render
PLATE_W = 520.0
PLATE_D = 450.0
PLATE_T = 12.0
PLATE_CORNER_R = 25.0
WIRE_D = 50.0
WIRE_BEND_R = 100.0
WIRE_END_FILLET = 10.0
# Wire centreline as (x, z) corners from the wire base; every corner gets WIRE_BEND_R
WIRE_PATH = [(0, 0), (0, 190), (300, 400), (-320, 600), (-20, 800), (-20, 900)]
WIRE_BASE_Y = 70.0  # wire base sits this far behind the plate centre
RING_OD = 200.0
RING_SECTION_D = 35.0

# Scene layout (cup centre at the origin, +Y away from the viewer of the render)
CUBE_POS = (320.0, -340.0)
STAND_POS = (835.0, 20.0)
# Tubes leaning in the cup: (colour, bottom-centre xy, top-centre xy). Each one
# rests on the cup floor, and none touch each other or the cup wall.
TUBES = [
    ("red", (-49.8, 60.6), (-286.6, 31.2)),
    ("blue", (-19.9, -106.7), (-187.8, 131.6)),
    ("blue", (15.7, -161.3), (-193.9, 53.4)),
    ("blue", (-68.7, -41.8), (-72.7, 192.1)),
    ("red", (-11.2, 96.9), (-25.4, 217.7)),
    ("red", (1.0, 45.3), (-77.5, -129.6)),
    ("red", (94.7, 103.7), (31.0, -102.6)),
    ("blue", (41.8, 95.9), (-5.8, -177.1)),
    ("red", (-15.9, -18.5), (274.5, -51.3)),
    ("blue", (13.7, 146.4), (299.2, -5.3)),
]

GOLD = Color(0.85, 0.65, 0.13)
GRAY = Color(0.5, 0.5, 0.5)
LIGHT_GRAY = Color(0.62, 0.62, 0.62)
BLACK = Color(0.05, 0.05, 0.05)
RED = Color(0.8, 0.0, 0.0)
BLUE = Color(0.0, 0.0, 0.8)


def tag(shape, label, color):
    shape.label = label
    shape.color = color
    return shape


# --------------------------------------------------------------------------- cube


def cube_body():
    """200 mm cube with all 12 edges chamfered, sitting on z = 0."""
    h = CUBE_SIZE / 2
    c = CUBE_CHAMFER
    octagon = Polygon(
        (-h + c, -h), (h - c, -h), (h, -h + c), (h, h - c),
        (h - c, h), (-h + c, h), (-h, h - c), (-h, -h + c),
        align=None,
    )
    body = extrude(Plane.XY * octagon, h, both=True)
    body &= extrude(Plane.YZ * octagon, h, both=True)
    body &= extrude(Plane.XZ * octagon, h, both=True)
    return Pos(0, 0, h) * body


def cube_face_planes():
    """Outward-facing planes on each flat face; local x is 'right', y is 'up'."""
    h = CUBE_SIZE / 2
    return {
        "front": Plane((0, -h, h), x_dir=(1, 0, 0), z_dir=(0, -1, 0)),
        "right": Plane((h, 0, h), x_dir=(0, 1, 0), z_dir=(1, 0, 0)),
        "back": Plane((0, h, h), x_dir=(-1, 0, 0), z_dir=(0, 1, 0)),
        "left": Plane((-h, 0, h), x_dir=(0, -1, 0), z_dir=(-1, 0, 0)),
        "top": Plane((0, 0, 2 * h), x_dir=(1, 0, 0), z_dir=(0, 0, 1)),
        "bottom": Plane((0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 0, -1)),
    }


def logo_sketch():
    upper = Polygon(*LOGO_HALF, align=None)
    lower = Polygon(*[(-x, -y) for x, y in LOGO_HALF], align=None)
    return upper + lower


def marker_sketch(corners):
    """corners maps (sx, sy) in {-1, 1}^2 to 'L' (bracket) or 'S' (solid square)."""
    o = MARKER_SQUARE / 2
    sketch = None
    for (sx, sy), kind in corners.items():
        if kind == "S":
            shapes = [Pos(sx * (o - MARKER_SIZE / 2), sy * (o - MARKER_SIZE / 2))
                      * Rectangle(MARKER_SIZE, MARKER_SIZE)]
        else:
            shapes = [
                Pos(sx * (o - MARKER_SIZE / 2), sy * (o - MARKER_ARM / 2))
                * Rectangle(MARKER_SIZE, MARKER_ARM),
                Pos(sx * (o - MARKER_ARM / 2), sy * (o - MARKER_SIZE / 2))
                * Rectangle(MARKER_ARM, MARKER_SIZE),
            ]
        for s in shapes:
            sketch = s if sketch is None else sketch + s
    return sketch


def barcode_sketch():
    module = BARCODE_W / len(BARCODE_PATTERN)
    sketch = None
    i = 0
    while i < len(BARCODE_PATTERN):
        j = i
        while j < len(BARCODE_PATTERN) and BARCODE_PATTERN[j] == BARCODE_PATTERN[i]:
            j += 1
        if BARCODE_PATTERN[i] == "1":
            x = -BARCODE_W / 2 + module * (i + j) / 2
            bar = Pos(x, 0) * Rectangle(module * (j - i), BARCODE_H)
            sketch = bar if sketch is None else sketch + bar
        i = j
    return sketch


def cube(with_fiducials=False):
    """The cube as an assembly: gold body plus black decals."""
    planes = cube_face_planes()
    decals = []
    for side in ("front", "right", "back", "left"):
        decals.append(extrude(planes[side] * logo_sketch(), DECAL_THICKNESS))
    if with_fiducials:
        side_markers = {(-1, 1): "L", (1, 1): "L", (-1, -1): "L", (1, -1): "S"}
        for side in ("front", "right", "back", "left"):
            decals.append(extrude(planes[side] * marker_sketch(side_markers), DECAL_THICKNESS))
        top = {(-1, 1): "S", (1, 1): "L", (-1, -1): "L", (1, -1): "S"}
        decals.append(extrude(planes["top"] * marker_sketch(top), DECAL_THICKNESS))
        bottom = {(-1, 1): "L", (1, 1): "S", (-1, -1): "S", (1, -1): "L"}
        decals.append(extrude(planes["bottom"] * marker_sketch(bottom), DECAL_THICKNESS))
        decals.append(extrude(planes["bottom"] * barcode_sketch(), DECAL_THICKNESS))
    decal = decals[0]
    for d in decals[1:]:
        decal += d
    return Compound(
        label="cube_200",
        children=[tag(cube_body(), "cube_body", GOLD), tag(decal, "cube_decals", BLACK)],
    )


# ----------------------------------------------------------------- cup and tubes


def cup():
    outer = Pos(0, 0, CUP_HEIGHT / 2) * Cylinder(CUP_OD / 2, CUP_HEIGHT)
    inner = Pos(0, 0, CUP_FLOOR + CUP_HEIGHT / 2) * Cylinder(CUP_OD / 2 - CUP_WALL, CUP_HEIGHT)
    return outer - inner


def tube():
    """Open tube standing on z = 0 along +Z."""
    return Pos(0, 0, TUBE_LENGTH / 2) * (
        Cylinder(TUBE_OD / 2, TUBE_LENGTH) - Cylinder(TUBE_ID / 2, TUBE_LENGTH)
    )


def tube_location(bottom_xy, top_xy):
    """Place a tube so it leans from bottom_xy to top_xy with its lower rim on the cup floor."""
    dx, dy = top_xy[0] - bottom_xy[0], top_xy[1] - bottom_xy[1]
    sin_tilt = math.hypot(dx, dy) / TUBE_LENGTH
    cos_tilt = math.sqrt(1 - sin_tilt**2)
    z0 = CUP_FLOOR + TUBE_OD / 2 * sin_tilt
    direction = (dx / TUBE_LENGTH, dy / TUBE_LENGTH, cos_tilt)
    return Location(Plane((bottom_xy[0], bottom_xy[1], z0), z_dir=direction))


def tube_cup():
    children = [tag(cup(), "cup", GRAY)]
    for i, (color, bottom, top) in enumerate(TUBES, start=1):
        # Fresh solid per tube: shared geometry would be merged into one STEP part
        # and lose the per-tube name and colour.
        t = tag(tube_location(bottom, top) * tube(), f"tube_{i:02d}_{color}",
                RED if color == "red" else BLUE)
        children.append(t)
    return Compound(label="tube_cup", children=children)


# -------------------------------------------------------------- buzz-wire stand


def base_plate():
    return extrude(RectangleRounded(PLATE_W, PLATE_D, PLATE_CORNER_R), PLATE_T)


def wire():
    """S-shaped rod, base of its centreline at the origin, swept along +Z in the XZ plane."""
    path = FilletPolyline(*[(x, 0, z) for x, z in WIRE_PATH], radius=WIRE_BEND_R)
    profile = Plane(path @ 0, z_dir=path % 0) * Circle(WIRE_D / 2)
    rod = sweep(profile, path=path)
    top_face = rod.faces().sort_by(Axis.Z)[-1]
    return fillet(top_face.edges(), WIRE_END_FILLET)


def ring():
    return Torus((RING_OD - RING_SECTION_D) / 2, RING_SECTION_D / 2)


def buzz_wire_stand():
    """Plate centred on the origin with the wire and the ring resting at its base."""
    base = Pos(0, WIRE_BASE_Y, PLATE_T)
    return Compound(
        label="buzz_wire_stand",
        children=[
            tag(base_plate(), "base_plate", LIGHT_GRAY),
            tag(base * wire(), "wire", LIGHT_GRAY),
            tag(base * Pos(0, 0, RING_SECTION_D / 2) * ring(), "ring", GOLD),
        ],
    )


# ------------------------------------------------------------------------ scene


def scene():
    return Compound(
        label="game_elements_scene",
        children=[
            tube_cup(),
            Pos(*CUBE_POS, 0) * cube(),
            Pos(*STAND_POS, 0) * buzz_wire_stand(),
        ],
    )


def main():
    step_dir = OUT_DIR / "step"
    stl_dir = OUT_DIR / "stl"
    step_dir.mkdir(exist_ok=True)
    stl_dir.mkdir(exist_ok=True)

    parts = {
        "cube_200": tag(cube_body(), "cube_body", GOLD),
        "cup": tag(cup(), "cup", GRAY),
        "tube": tag(tube(), "tube", RED),
        "base_plate": tag(base_plate(), "base_plate", LIGHT_GRAY),
        "wire": tag(wire(), "wire", LIGHT_GRAY),
        "ring": tag(ring(), "ring", GOLD),
    }
    for name, shape in parts.items():
        export_stl(shape, stl_dir / f"{name}.stl", tolerance=0.1, angular_tolerance=0.15)
        print(f"stl/{name}.stl")

    assemblies = {
        "cube_200": cube(),
        "cube_200_fiducials": cube(with_fiducials=True),
        "cup": parts["cup"],
        "tube": parts["tube"],
        "tube_cup": tube_cup(),
        "base_plate": parts["base_plate"],
        "wire": parts["wire"],
        "ring": parts["ring"],
        "buzz_wire_stand": buzz_wire_stand(),
        "scene": scene(),
    }
    for name, shape in assemblies.items():
        export_step(shape, step_dir / f"{name}.step", timestamp="2026-01-01T00:00:00")
        print(f"step/{name}.step")

    export_gltf(assemblies["scene"], OUT_DIR / "scene.glb", binary=True,
                linear_deflection=0.3, angular_deflection=0.2)
    print("scene.glb")


if __name__ == "__main__":
    main()
