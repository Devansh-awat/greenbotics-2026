"""
WRO FE 2026 — Pillar Numbering Diagnostic / Alignment Tool
=============================================================
Purpose: Draw all 21 numbered pillar positions (per the user's own
annotation convention) onto the base playfield image so it can be
visually compared against the hand-annotated reference photo
(Field_annotations_va.jpeg) BEFORE any color-assignment logic is built.

Numbering convention confirmed by user:
  - Bottom section = PARKING side. Only 3 positions exist here (no
    outer row - blocks on parking side are never placed in the outer
    row). Numbered left-to-right: 3, 4, 5.
  - Right section (going from the bottom corner up to the top corner):
      inner = 6, 8, 10   outer = 7, 9, 11   (paired slot by slot)
  - Top section (going from the right corner to the left corner):
      inner = 12, 14, 16   outer = 13, 15, 17
  - Left section (going from the top corner down to the bottom corner):
      inner = 18, 20, 2    outer = 19, 21, 1

  "Inner" = towards the WRO logo / inner track wall.
  "Outer" = towards the outer boundary wall.

This script does NOT do any red/green assignment yet. It just labels
each of the 21 positions with its number, color-coded by row
(inner=blue, outer=orange) so we can confirm alignment with the
photo first.
"""

import os
from PIL import Image, ImageDraw, ImageFont

BASE_IMAGE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "input_config", "WRO-2025_FutureEngineers_Playfield.jpg"
)
OUTPUT_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "input_config", "diagnostic_pillar_numbering.png"
)

# Re-used verbatim from generate_tough_configs.py so we stay consistent
# with the existing coordinate system already calibrated to this image.
SECTION_PILLAR_COORDS_OUTWARD = {
    "Top": {"p1": (710, 320), "p2": (1025, 320), "p3": (1345, 320)},
    "Right": {"p1": (1725, 710), "p2": (1725, 1025), "p3": (1725, 1345)},
    "Bottom": {"p1": (1345, 1725), "p2": (1025, 1725), "p3": (710, 1725)},
    "Left": {"p1": (320, 1345), "p2": (320, 1025), "p3": (320, 710)},
}
SECTION_PILLAR_COORDS_INWARD = {
    "Top": {"p1": (710, 450), "p2": (1025, 450), "p3": (1345, 450)},
    "Right": {"p1": (1600, 710), "p2": (1600, 1025), "p3": (1600, 1345)},
    "Bottom": {"p1": (1345, 1600), "p2": (1025, 1600), "p3": (710, 1600)},
    "Left": {"p1": (450, 1345), "p2": (450, 1025), "p3": (450, 710)},
}

# Parking lot line coordinates - reused verbatim from generate_tough_configs.py
PARKING_WIDTH = 160
PARKING_LENGTH = 100
PARKING_LOT_COORDS = {
    "Top": [
        ((710, 70), (710, 70 + PARKING_LENGTH)),
        ((710 + PARKING_WIDTH, 70), (710 + PARKING_WIDTH, 70 + PARKING_LENGTH)),
    ],
    "Right": [
        ((1975, 710), (1975 - PARKING_LENGTH, 710)),
        ((1975, 710 + PARKING_WIDTH), (1975 - PARKING_LENGTH, 710 + PARKING_WIDTH)),
    ],
    "Bottom": [
        ((1340, 1975), (1340, 1975 - PARKING_LENGTH)),
        ((1340 - PARKING_WIDTH, 1975), (1340 - PARKING_WIDTH, 1975 - PARKING_LENGTH)),
    ],
    "Left": [
        ((70, 1340), (70 + PARKING_LENGTH, 1340)),
        ((70, 1340 - PARKING_WIDTH), (70 + PARKING_LENGTH, 1340 - PARKING_WIDTH)),
    ],
}
PARKING_LINE_COLOR = "#F702F9"

# ---------------------------------------------------------------------
# Mapping: user's pillar NUMBER -> (section, slot key p1/p2/p3, row)
#
# The original coordinate dict's p1/p2/p3 are defined "in clockwise
# order" per section. The user's numbering sweeps counter-clockwise
# (Bottom -> Right -> Top -> Left), and within each side follows a
# specific physical direction. We map user numbers directly onto the
# physical p1/p2/p3 slots based on which corner they are nearest,
# rather than assuming direction - this will be verified visually.
#
# Bottom (parking): p1=nearest Right corner, p2=middle, p3=nearest Left
#   corner (per original "clockwise" definition for Bottom, since
#   clockwise travel along Bottom goes Left-corner -> Right-corner is
#   backwards; original dict lists p1=(1345,1725) which is the RIGHT
#   side of Bottom, p3=(710,1725) which is the LEFT side of Bottom).
#   User said Bottom numbers 3,4,5 go LEFT to RIGHT.
#   So: left-most = 3, middle = 4, right-most = 5.
#   left-most physically = p3 (710,1725); right-most = p1 (1345,1725)
#   => 3 -> p3, 4 -> p2, 5 -> p1   (single row, positioned on the
#      INWARD coordinate set - closer to the logo/inner wall - per
#      user correction; visually level with the other sections'
#      inner row).
#
# Right section: p1=(1725,710) [nearest Top corner], p3=(1725,1345)
#   [nearest Bottom corner] per original clockwise definition.
#   User: numbering goes from the BOTTOM corner up to the TOP corner:
#     inner = 6, 8, 10 ; outer = 7, 9, 11
#   nearest-bottom slot = p3, middle = p2, nearest-top slot = p1
#   => slot p3 (nearest bottom): inner=6, outer=7
#      slot p2 (middle):          inner=8, outer=9
#      slot p1 (nearest top):     inner=10, outer=11
#
# Top section: p1=(710,320) [nearest Left corner], p3=(1345,320)
#   [nearest Right corner] per original clockwise definition.
#   User: numbering goes from the RIGHT corner to the LEFT corner:
#     inner = 12, 14, 16 ; outer = 13, 15, 17
#   nearest-right slot = p3, middle = p2, nearest-left slot = p1
#   => slot p3 (nearest right): inner=12, outer=13
#      slot p2 (middle):         inner=14, outer=15
#      slot p1 (nearest left):   inner=16, outer=17
#
# Left section: p1=(320,1345) [nearest Bottom corner], p3=(320,710)
#   [nearest Top corner] per original clockwise definition.
#   User: numbering goes from the TOP corner down to the BOTTOM corner:
#     inner = 18, 20, 2 ; outer = 19, 21, 1
#   nearest-top slot = p3, middle = p2, nearest-bottom slot = p1
#   => slot p3 (nearest top):    inner=18, outer=19
#      slot p2 (middle):         inner=20, outer=21
#      slot p1 (nearest bottom): inner=2,  outer=1
# ---------------------------------------------------------------------

PILLAR_NUMBER_MAP = {
    # number: (section, slot, row)   row in {"Inner", "Outer"}
    # Bottom (parking) pillars sit on the INNER row coordinate set
    # (closer to the logo), confirmed by user.
    3:  ("Bottom", "p3", "Inner"),
    4:  ("Bottom", "p2", "Inner"),
    5:  ("Bottom", "p1", "Inner"),

    6:  ("Right", "p3", "Inner"),
    7:  ("Right", "p3", "Outer"),
    8:  ("Right", "p2", "Inner"),
    9:  ("Right", "p2", "Outer"),
    10: ("Right", "p1", "Inner"),
    11: ("Right", "p1", "Outer"),

    12: ("Top", "p3", "Inner"),
    13: ("Top", "p3", "Outer"),
    14: ("Top", "p2", "Inner"),
    15: ("Top", "p2", "Outer"),
    16: ("Top", "p1", "Inner"),
    17: ("Top", "p1", "Outer"),

    18: ("Left", "p3", "Inner"),
    19: ("Left", "p3", "Outer"),
    20: ("Left", "p2", "Inner"),
    21: ("Left", "p2", "Outer"),
    2:  ("Left", "p1", "Inner"),
    1:  ("Left", "p1", "Outer"),
}


def get_pixel_coords(number):
    section, slot, row = PILLAR_NUMBER_MAP[number]
    coord_dict = SECTION_PILLAR_COORDS_INWARD if row == "Inner" else SECTION_PILLAR_COORDS_OUTWARD
    return coord_dict[section][slot]


def draw_parking_lot(draw, section="Bottom"):
    """Draws the parking lot lines for the given section and labels
    the left line 'L' and the right line 'R' (as seen facing the
    section from outside the track), matching the user's photo."""
    coords = PARKING_LOT_COORDS[section]
    draw.line(coords[0], fill=PARKING_LINE_COLOR, width=14)
    draw.line(coords[1], fill=PARKING_LINE_COLOR, width=14)
    return coords


def main():
    img = Image.open(BASE_IMAGE_PATH).convert("RGBA")
    draw = ImageDraw.Draw(img)

    try:
        font = ImageFont.truetype("arial.ttf", 36)
        label_font = ImageFont.truetype("arial.ttf", 44)
    except Exception:
        font = ImageFont.load_default()
        label_font = font

    radius = 30
    for number in range(1, 22):
        x, y = get_pixel_coords(number)
        section, slot, row = PILLAR_NUMBER_MAP[number]
        if section == "Bottom":
            color = "#8e44ad"  # purple = parking-side single row
        else:
            color = "#1f6feb" if row == "Inner" else "#e8790c"  # blue=inner, orange=outer

        bbox = [x - radius, y - radius, x + radius, y + radius]
        draw.ellipse(bbox, fill=color, outline="black", width=3)

        text = str(number)
        tb = draw.textbbox((0, 0), text, font=font)
        tw, th = tb[2] - tb[0], tb[3] - tb[1]
        draw.text((x - tw / 2, y - th / 2 - 4), text, fill="white", font=font)

    # --- Parking lines (pink) + L/R labels, Bottom section ---
    coords = draw_parking_lot(draw, "Bottom")
    # PARKING_LOT_COORDS["Bottom"][0] is at x=1340 (the RIGHT line),
    # PARKING_LOT_COORDS["Bottom"][1] is at x=1340-160=1180 (the LEFT line).
    right_line, left_line = coords[0], coords[1]
    lx = left_line[0][0]
    rx = right_line[0][0]
    label_y = left_line[0][1] - 60  # just above the parking box, inside the field

    l_text, r_text = "L", "R"
    l_tb = draw.textbbox((0, 0), l_text, font=label_font)
    r_tb = draw.textbbox((0, 0), r_text, font=label_font)
    l_tw = l_tb[2] - l_tb[0]
    r_tw = r_tb[2] - r_tb[0]
    draw.text((lx - l_tw / 2, label_y), l_text, fill="#F702F9", font=label_font)
    draw.text((rx - r_tw / 2, label_y), r_text, fill="#F702F9", font=label_font)

    # Legend
    legend_y = 40
    draw.ellipse([40, legend_y, 40 + 40, legend_y + 40], fill="#1f6feb", outline="black", width=2)
    draw.text((90, legend_y + 5), "Inner (towards logo)", fill="black", font=font)
    draw.ellipse([40, legend_y + 55, 40 + 40, legend_y + 95], fill="#e8790c", outline="black", width=2)
    draw.text((90, legend_y + 60), "Outer (towards wall)", fill="black", font=font)
    draw.ellipse([40, legend_y + 110, 40 + 40, legend_y + 150], fill="#8e44ad", outline="black", width=2)
    draw.text((90, legend_y + 115), "Parking side (single row)", fill="black", font=font)
    draw.line([(40, legend_y + 178), (80, legend_y + 178)], fill=PARKING_LINE_COLOR, width=8)
    draw.text((90, legend_y + 165), "Parking lot boundary (L / R)", fill="black", font=font)

    # Mark bottom section as "Parking side" for clarity
    draw.text((870, 1850), "PARKING SIDE (Bottom)", fill="black", font=font)

    img.save(OUTPUT_PATH)
    print(f"Saved diagnostic image to: {OUTPUT_PATH}")


if __name__ == "__main__":
    main()
