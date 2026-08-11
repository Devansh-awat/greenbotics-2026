"""
WRO FE 2026 — Toughest Obstacle Challenge Configuration Generator
Generates images for the 5 hardest possible configurations for competition day testing.

Based on:  WRO 2026 official rules.
"""

import os
import sys

from PIL import Image, ImageDraw, ImageFont

# --- CONFIGURATION & CONSTANTS ---

BASE_IMAGE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "WRO-2025_FutureEngineers_Playfield.jpg"
)

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "tough_configs")

PILLAR_RADIUS = 28
PARKING_WIDTH = 160
PARKING_LENGTH = 100
FONT_SIZE = 40
TITLE_FONT_SIZE = 50

# Colors for drawing
COLORS = {
    "Red": "#d90429",
    "Green": "#008000",
    "Parking": "#F702F9",
    "Text": "#FFFFFF",
    "TextBG": "#000000",
    "Arrow": "#000000",
    "Title": "#FFFFFF",
    "TitleBG": "#333333",
}

# --- COORDINATES (based on a 2134x2134 image) ---

# Positions for pillars within each straight section
# 'p1' is first in clockwise order, 'p2' is middle, 'p3' is last.
SECTION_PILLAR_COORDS_OUTWARD = {
    "Top": {"p1": (710, 320), "p2": (1025, 320), "p3": (1345, 320)},
    "Right": {"p1": (1725, 710), "p2": (1725, 1025), "p3": (1725, 1345)},
    "Bottom": {"p1": (1345, 1725), "p2": (1025, 1725), "p3": (710, 1725)},
    "Left": {"p1": (320, 1345), "p2": (320, 1025), "p3": (320, 710)},
}
# Coords for when pillars are moved inward
SECTION_PILLAR_COORDS_INWARD = {
    "Top": {"p1": (710, 450), "p2": (1025, 450), "p3": (1345, 450)},
    "Right": {"p1": (1600, 710), "p2": (1600, 1025), "p3": (1600, 1345)},
    "Bottom": {"p1": (1345, 1600), "p2": (1025, 1600), "p3": (710, 1600)},
    "Left": {"p1": (450, 1345), "p2": (450, 1025), "p3": (450, 710)},
}

# Coordinates for drawing the two parallel lines of the parking lot
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

# Coordinates for direction arrows on the track
ARROW_COORDS = {
    "Clockwise": [
        {"line": ((850, 385), (1200, 385)), "head": [(1200, 365), (1240, 385), (1200, 405)]},
        {"line": ((1665, 850), (1665, 1200)), "head": [(1645, 1200), (1665, 1240), (1685, 1200)]},
        {"line": ((1200, 1665), (850, 1665)), "head": [(850, 1645), (810, 1665), (850, 1685)]},
        {"line": ((385, 1200), (385, 850)), "head": [(365, 850), (385, 810), (405, 850)]},
    ],
    "Counter-Clockwise": [
        {"line": ((1200, 385), (850, 385)), "head": [(850, 365), (810, 385), (850, 405)]},
        {"line": ((1665, 1200), (1665, 850)), "head": [(1645, 850), (1665, 810), (1685, 850)]},
        {"line": ((850, 1665), (1200, 1665)), "head": [(1200, 1645), (1240, 1665), (1200, 1685)]},
        {"line": ((385, 850), (385, 1200)), "head": [(365, 1200), (385, 1240), (405, 1200)]},
    ],
}

# Card layouts from official WRO rules
CARD_LAYOUTS = {
    1: {"pillars": [("p3", "Green", "Inner")]},
    2: {"pillars": [("p3", "Red", "Inner")]},
    3: {"pillars": [("p2", "Green", "Inner")]},
    4: {"pillars": [("p2", "Red", "Inner")]},
    5: {"pillars": [("p1", "Green", "Inner")]},
    6: {"pillars": [("p1", "Red", "Inner")]},
    7: {"pillars": [("p3", "Green", "Outer")]},
    8: {"pillars": [("p3", "Red", "Outer")]},
    9: {"pillars": [("p2", "Green", "Outer")]},
    10: {"pillars": [("p2", "Red", "Outer")]},
    11: {"pillars": [("p1", "Green", "Inner")]},
    12: {"pillars": [("p1", "Red", "Inner")]},
    13: {"pillars": [("p3", "Green", "Outer"), ("p1", "Green", "Inner")]},
    14: {"pillars": [("p3", "Green", "Outer"), ("p1", "Red", "Inner")]},
    15: {"pillars": [("p3", "Red", "Outer"), ("p1", "Green", "Inner")]},
    16: {"pillars": [("p3", "Green", "Outer"), ("p1", "Red", "Inner")]},
    17: {"pillars": [("p3", "Red", "Outer"), ("p1", "Green", "Inner")]},
    18: {"pillars": [("p3", "Red", "Outer"), ("p1", "Red", "Inner")]},
    19: {"pillars": [("p3", "Green", "Inner"), ("p1", "Green", "Outer")]},
    20: {"pillars": [("p3", "Green", "Inner"), ("p1", "Red", "Outer")]},
    21: {"pillars": [("p3", "Red", "Inner"), ("p1", "Green", "Outer")]},
    22: {"pillars": [("p3", "Green", "Inner"), ("p1", "Red", "Outer")]},
    23: {"pillars": [("p3", "Red", "Inner"), ("p1", "Green", "Outer")]},
    24: {"pillars": [("p3", "Red", "Inner"), ("p1", "Red", "Outer")]},
    25: {"pillars": [("p3", "Green", "Inner"), ("p1", "Green", "Inner")]},
    26: {"pillars": [("p3", "Green", "Inner"), ("p1", "Red", "Inner")]},
    27: {"pillars": [("p3", "Red", "Inner"), ("p1", "Green", "Inner")]},
    28: {"pillars": [("p3", "Green", "Inner"), ("p1", "Red", "Inner")]},
    29: {"pillars": [("p3", "Red", "Inner"), ("p1", "Green", "Inner")]},
    30: {"pillars": [("p3", "Red", "Inner"), ("p1", "Red", "Inner")]},
    31: {"pillars": [("p3", "Green", "Outer"), ("p1", "Green", "Outer")]},
    32: {"pillars": [("p3", "Green", "Outer"), ("p1", "Red", "Outer")]},
    33: {"pillars": [("p3", "Red", "Outer"), ("p1", "Green", "Outer")]},
    34: {"pillars": [("p3", "Green", "Outer"), ("p1", "Red", "Outer")]},
    35: {"pillars": [("p3", "Red", "Outer"), ("p1", "Green", "Outer")]},
    36: {"pillars": [("p3", "Red", "Outer"), ("p1", "Red", "Outer")]},
}


# --- DRAWING FUNCTIONS ---

def draw_pillar(draw, center_xy, color):
    """Draws a colored circle (pillar) on the image."""
    x, y = center_xy
    bbox = [x - PILLAR_RADIUS, y - PILLAR_RADIUS, x + PILLAR_RADIUS, y + PILLAR_RADIUS]
    draw.ellipse(bbox, fill=COLORS[color], outline="black", width=3)
    # Add a white letter for clarity
    letter = "R" if color == "Red" else "G"
    try:
        font = ImageFont.truetype("arial.ttf", 24)
    except:
        font = ImageFont.load_default()
    bbox_text = draw.textbbox((0, 0), letter, font=font)
    tw = bbox_text[2] - bbox_text[0]
    th = bbox_text[3] - bbox_text[1]
    draw.text((x - tw//2, y - th//2 - 2), letter, fill="white", font=font)


def draw_parking_lot(draw, section):
    """Draws the parking lot lines for a given section."""
    coords = PARKING_LOT_COORDS[section]
    draw.line(coords[0], fill=COLORS["Parking"], width=12)
    draw.line(coords[1], fill=COLORS["Parking"], width=12)


def draw_direction_arrows(draw, direction):
    """Draws arrows on the track to indicate driving direction."""
    arrows = ARROW_COORDS[direction]
    for arrow in arrows:
        draw.line(arrow["line"], fill=COLORS["Arrow"], width=15)
        draw.polygon(arrow["head"], fill=COLORS["Arrow"])


def draw_title(image, draw, title, subtitle):
    """Draws a title bar at the top of the image."""
    # Draw semi-transparent bar at top
    bar_height = 120
    overlay = Image.new('RGBA', (image.width, bar_height), (40, 40, 40, 220))
    image.paste(overlay, (0, 0), overlay)
    
    try:
        title_font = ImageFont.truetype("arial.ttf", TITLE_FONT_SIZE)
        subtitle_font = ImageFont.truetype("arial.ttf", 30)
    except:
        title_font = ImageFont.load_default()
        subtitle_font = ImageFont.load_default()
    
    draw.text((30, 10), title, fill="#FFFFFF", font=title_font)
    draw.text((30, 70), subtitle, fill="#CCCCCC", font=subtitle_font)


def draw_info_box(image, draw, info_lines, position="bottom"):
    """Draws an info box with configuration details."""
    box_height = len(info_lines) * 35 + 20
    box_width = 600
    
    if position == "bottom":
        y_start = image.height - box_height - 20
        x_start = 20
    else:
        y_start = 140
        x_start = 20
    
    # Semi-transparent background
    overlay = Image.new('RGBA', (box_width, box_height), (0, 0, 0, 200))
    image.paste(overlay, (x_start, y_start), overlay)
    
    try:
        font = ImageFont.truetype("arial.ttf", 26)
    except:
        font = ImageFont.load_default()
    
    for i, line in enumerate(info_lines):
        draw.text((x_start + 15, y_start + 10 + i * 35), line, fill="#FFFFFF", font=font)


def generate_configuration_image(config, output_filename):
    """Loads the base image and draws the specified configuration on it."""
    try:
        base_image = Image.open(BASE_IMAGE_PATH).convert("RGBA")
    except FileNotFoundError:
        print(f"Error: Could not find base image at: {BASE_IMAGE_PATH}")
        return False

    draw = ImageDraw.Draw(base_image)

    # Draw Direction Arrows
    draw_direction_arrows(draw, config["direction"])

    # Draw Parking Lot
    draw_parking_lot(draw, config["parking_section"])

    # Combine all section layouts
    all_sections_layout = config["other_sections_layout"].copy()
    all_sections_layout[config["single_sign_section"]] = config["single_sign_card"]

    # Draw all pillars from cards
    total_pillars = 0
    for section, card_num in all_sections_layout.items():
        layout = CARD_LAYOUTS[card_num]

        for pillar_data in layout["pillars"]:
            pillar_pos, pillar_color, pillar_row = pillar_data

            # Determine if the pillar is inward or outward
            if section == config["parking_section"] or pillar_row == "Inner":
                is_inward = True
            else:
                is_inward = False

            pillar_coords = (
                SECTION_PILLAR_COORDS_INWARD if is_inward
                else SECTION_PILLAR_COORDS_OUTWARD
            )
            center_xy = pillar_coords[section][pillar_pos]
            draw_pillar(draw, center_xy, pillar_color)
            total_pillars += 1

    # Rotate so parking is at bottom
    parking_section = config["parking_section"]
    rotation_angle = 0
    if parking_section == "Right":
        rotation_angle = 270
    elif parking_section == "Top":
        rotation_angle = 180
    elif parking_section == "Left":
        rotation_angle = 90

    if rotation_angle != 0:
        base_image = base_image.rotate(rotation_angle, expand=True)

    # Redraw on rotated image for title and info
    draw = ImageDraw.Draw(base_image)
    
    # Draw title
    draw_title(base_image, draw, config["title"], config["subtitle"])
    
    # Draw info box
    info_lines = [
        f"Direction: {config['direction']}",
        f"Parking: {config['parking_section']} section",
        f"Total Pillars: {total_pillars}",
        f"Difficulty: {config['difficulty']}",
    ]
    draw_info_box(base_image, draw, info_lines)

    # Save
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    output_path = os.path.join(OUTPUT_DIR, output_filename)
    base_image.save(output_path)
    print(f"  Generated: {output_path}")
    return True


# --- THE 6 TOUGHEST CONFIGURATIONS (3 CW + 3 CCW) ---
# Difficulty is based on: Color + Position + Direction determines
# whether robot faces a ~250mm tight gap or a ~750mm easy gap.
#
# CLOCKWISE:  Green/Outer = TIGHT, Red/Inner = TIGHT
# CCW:        Green/Inner = TIGHT, Red/Outer = TIGHT
#
# "Double TIGHT" cards (both pillars force tight wall gaps):
#   CW hard:  14, 16, 21, 23, 30, 31
#   CCW hard: 15, 17, 20, 22, 25, 36

TOUGH_CONFIGS = [
    # === CLOCKWISE CONFIGS ===
    {
        "title": "CW-1: WALL SCRAPER",
        "subtitle": "CW: All 7 pillars force tight ~250mm wall gaps",
        "difficulty": "★★★★★ EXTREME (CW)",
        "direction": "Clockwise",
        "single_sign_section": "Bottom",
        "single_sign_card": 7,  # Green @ p3/Outer (tight in CW!)
        "parking_section": "Bottom",
        "other_sections_layout": {
            "Left": 14,   # p3=Green/Outer(tight) + p1=Red/Inner(tight) = DOUBLE TIGHT
            "Top": 30,    # p3=Red/Inner(tight) + p1=Red/Inner(tight) = DOUBLE TIGHT
            "Right": 31,  # p3=Green/Outer(tight) + p1=Green/Outer(tight) = DOUBLE TIGHT
        },
        "filename": "config_CW1_wall_scraper.png",
    },
    {
        "title": "CW-2: THE ZIGZAG",
        "subtitle": "CW: Alternates outer-wall squeeze then inner-wall squeeze",
        "difficulty": "★★★★★ EXTREME (CW)",
        "direction": "Clockwise",
        "single_sign_section": "Top",
        "single_sign_card": 6,  # Red @ p1/Inner (tight in CW!)
        "parking_section": "Top",
        "other_sections_layout": {
            "Right": 14,   # p3=Green/Outer(tight) + p1=Red/Inner(tight)
            "Bottom": 16,  # p3=Green/Outer(tight) + p1=Red/Inner(tight)
            "Left": 14,    # p3=Green/Outer(tight) + p1=Red/Inner(tight)
        },
        "filename": "config_CW2_zigzag.png",
    },
    {
        "title": "CW-3: CORNER EXIT TRAP",
        "subtitle": "CW: Red/Inner at p1 = tight gap after every turn",
        "difficulty": "★★★★☆ VERY HARD (CW)",
        "direction": "Clockwise",
        "single_sign_section": "Left",
        "single_sign_card": 12,  # Red @ p1/Inner (tight in CW!)
        "parking_section": "Left",
        "other_sections_layout": {
            "Top": 18,     # p3=Red/Outer(easy CW) + p1=Red/Inner(tight)
            "Right": 30,   # p3=Red/Inner(tight) + p1=Red/Inner(tight)
            "Bottom": 16,  # p3=Green/Outer(tight) + p1=Red/Inner(tight)
        },
        "filename": "config_CW3_corner_exit.png",
    },
    # === COUNTER-CLOCKWISE CONFIGS ===
    {
        "title": "CCW-1: WALL SCRAPER",
        "subtitle": "CCW: All 7 pillars force tight ~250mm wall gaps",
        "difficulty": "★★★★★ EXTREME (CCW)",
        "direction": "Counter-Clockwise",
        "single_sign_section": "Top",
        "single_sign_card": 8,  # Red @ p3/Outer (tight in CCW!)
        "parking_section": "Top",
        "other_sections_layout": {
            "Right": 25,   # p3=Green/Inner(tight) + p1=Green/Inner(tight) = DOUBLE TIGHT
            "Bottom": 36,  # p3=Red/Outer(tight) + p1=Red/Outer(tight) = DOUBLE TIGHT
            "Left": 15,    # p3=Red/Outer(tight) + p1=Green/Inner(tight) = DOUBLE TIGHT
        },
        "filename": "config_CCW1_wall_scraper.png",
    },
    {
        "title": "CCW-2: THE ZIGZAG",
        "subtitle": "CCW: Alternates inner-wall squeeze then outer-wall squeeze",
        "difficulty": "★★★★★ EXTREME (CCW)",
        "direction": "Counter-Clockwise",
        "single_sign_section": "Bottom",
        "single_sign_card": 5,  # Green @ p1/Inner (tight in CCW!)
        "parking_section": "Bottom",
        "other_sections_layout": {
            "Left": 20,    # p3=Green/Inner(tight) + p1=Red/Outer(tight)
            "Top": 22,     # p3=Green/Inner(tight) + p1=Red/Outer(tight)
            "Right": 20,   # p3=Green/Inner(tight) + p1=Red/Outer(tight)
        },
        "filename": "config_CCW2_zigzag.png",
    },
    {
        "title": "CCW-3: FIRST STRAIGHT AMBUSH",
        "subtitle": "CCW: Hardest pillars in first section after parking",
        "difficulty": "★★★★☆ VERY HARD (CCW)",
        "direction": "Counter-Clockwise",
        "single_sign_section": "Left",
        "single_sign_card": 11,  # Green @ p1/Inner (tight in CCW!)
        "parking_section": "Left",
        "other_sections_layout": {
            "Bottom": 17,  # p3=Red/Outer(tight) + p1=Green/Inner(tight) — FIRST SECTION
            "Right": 36,   # p3=Red/Outer(tight) + p1=Red/Outer(tight)
            "Top": 25,     # p3=Green/Inner(tight) + p1=Green/Inner(tight)
        },
        "filename": "config_CCW3_first_straight.png",
    },
]


def main():
    print("=" * 60)
    print("WRO FE 2026 — Toughest Configuration Image Generator")
    print("=" * 60)
    print(f"\nBase image: {BASE_IMAGE_PATH}")
    print(f"Output dir: {OUTPUT_DIR}\n")
    
    if not os.path.exists(BASE_IMAGE_PATH):
        print(f"ERROR: Base image not found at: {BASE_IMAGE_PATH}")
        print("Please ensure the KMIDS playfield image exists.")
        sys.exit(1)
    
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    
    success_count = 0
    total = len(TOUGH_CONFIGS)
    for i, config in enumerate(TOUGH_CONFIGS, 1):
        print(f"\n[{i}/{total}] Generating: {config['title']}")
        result = generate_configuration_image(config, config["filename"])
        if result:
            success_count += 1
    
    print(f"\n{'=' * 60}")
    print(f"Done! Generated {success_count}/{total} images in: {OUTPUT_DIR}")
    print("=" * 60)


if __name__ == "__main__":
    main()
