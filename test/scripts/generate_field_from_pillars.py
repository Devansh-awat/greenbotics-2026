"""
WRO FE 2026 - Field Generator from Pillar-Number Input
=========================================================
Generates a playfield image by placing Red/Green pillars at specific
numbered positions (1-21), using the user's own field-annotation
convention (see Field_annotations_va.jpeg and markup_pillar_numbers.py
for the confirmed coordinate mapping).

INPUT FORMAT
------------
A single string (or list of tokens) of alternating <number> <color>
pairs, e.g.:

    "5 G 6 G 10 G 12 G 16 R 18 G 2 R 3 R"

  - <number> is 1-21, matching the physical pillar position per the
    field annotation photo.
  - <color> is "G" (Green) or "R" (Red) - case-insensitive.

Numbering convention (confirmed with user):
  - Bottom section = PARKING side. Only 3 positions: 3, 4, 5
    (left to right). No outer row on this side.
  - Right section (bottom-corner -> top-corner):
        inner = 6, 8, 10   outer = 7, 9, 11
  - Top section (right-corner -> left-corner):
        inner = 12, 14, 16   outer = 13, 15, 17
  - Left section (top-corner -> bottom-corner):
        inner = 18, 20, 2    outer = 19, 21, 1

  Inner = towards the WRO logo / inner track wall.
  Outer = towards the outer boundary wall.

USAGE
-----
    python generate_field_from_pillars.py "5 G 6 G 10 G 12 G 16 R 18 G 2 R 3 R"

    or import and call directly:
        from generate_field_from_pillars import generate_field_from_pillar_list
        generate_field_from_pillar_list("5 G 6 G 10 G 12 G 16 R 18 G 2 R 3 R",
                                         output_filename="my_config.png")
"""

import os
import re
import sys

from PIL import Image, ImageDraw, ImageFont

# --- CONFIGURATION & CONSTANTS (kept consistent with generate_tough_configs.py) ---

BASE_IMAGE_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "input_config", "WRO-2025_FutureEngineers_Playfield.jpg"
)
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pillar_configs")

PILLAR_RADIUS = 28
PARKING_WIDTH = 160
PARKING_LENGTH = 100

COLORS = {
    "R": "#d90429",
    "G": "#008000",
    "Parking": "#F702F9",
    "Title": "#FFFFFF",
}

COLOR_NAMES = {"R": "Red", "G": "Green"}

# --- COORDINATES (identical to generate_tough_configs.py / markup_pillar_numbers.py) ---

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

# --- CONFIRMED PILLAR NUMBER -> (section, slot, row) MAPPING ---
# Verified visually against Field_annotations_va.jpeg via
# markup_pillar_numbers.py before this file was written.

PILLAR_NUMBER_MAP = {
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

# Which numbers belong to the parking (Bottom) section - these never
# take an inner/outer color-row distinction; there's only one row.
PARKING_NUMBERS = {3, 4, 5}


def get_pixel_coords(number):
    """Return the (x, y) pixel coordinates for a given pillar number (1-21)."""
    if number not in PILLAR_NUMBER_MAP:
        raise ValueError(f"Invalid pillar number: {number}. Must be between 1 and 21.")
    section, slot, row = PILLAR_NUMBER_MAP[number]
    coord_dict = SECTION_PILLAR_COORDS_INWARD if row == "Inner" else SECTION_PILLAR_COORDS_OUTWARD
    return coord_dict[section][slot]


def parse_pillar_input(pillar_string):
    """
    Parses pillar configuration input into a list of (number, color)
    tuples, e.g. [(5, 'G'), (6, 'G'), ...].

    Accepts flexible formats, all equivalent:
        "5G 6G 10G 12G 16R 18G 2R 3R"   (number+color concatenated)
        "5 G 6 G 10 G 12 G 16 R 18 G 2 R 3 R"   (space separated)
        A mix of both is also fine.

    Also accepts a pre-split list of (number, color) tuples or tokens.
    """
    if not isinstance(pillar_string, str):
        # Already a list - could be list of tuples or flat tokens
        items = list(pillar_string)
        if items and isinstance(items[0], (tuple, list)):
            pairs = []
            for number, color in items:
                number = int(number)
                color = str(color).upper()
                if number not in PILLAR_NUMBER_MAP:
                    raise ValueError(f"Pillar number {number} is out of range (must be 1-21).")
                if color not in ("R", "G"):
                    raise ValueError(f"Invalid color '{color}' for pillar {number}. Use 'R' or 'G'.")
                pairs.append((number, color))
            _check_duplicates(pairs)
            return pairs
        tokens = items
    else:
        tokens = re.split(r"[\s,]+", pillar_string.strip())
        tokens = [t for t in tokens if t]

    # Normalize tokens: split any concatenated "<number><letter>" token
    # (e.g. "5G") into separate "5" and "G" tokens, so downstream logic
    # only ever deals with alternating number/color tokens.
    normalized = []
    for token in tokens:
        match = re.fullmatch(r"(\d+)([A-Za-z]+)", token)
        if match:
            normalized.append(match.group(1))
            normalized.append(match.group(2))
        else:
            normalized.append(token)
    tokens = normalized

    if len(tokens) % 2 != 0:
        raise ValueError(
            f"Malformed input: expected pairs of <number> <color>, "
            f"got {len(tokens)} tokens: {tokens}"
        )

    pairs = []
    for i in range(0, len(tokens), 2):
        num_token = tokens[i]
        color_token = tokens[i + 1].upper()

        try:
            number = int(num_token)
        except ValueError:
            raise ValueError(f"Invalid pillar number token: '{num_token}'")

        if number not in PILLAR_NUMBER_MAP:
            raise ValueError(f"Pillar number {number} is out of range (must be 1-21).")

        if color_token not in ("R", "G"):
            raise ValueError(f"Invalid color '{color_token}' for pillar {number}. Use 'R' or 'G'.")

        pairs.append((number, color_token))

    _check_duplicates(pairs)
    return pairs


def _check_duplicates(pairs):
    """Raises ValueError if the same pillar number appears more than once."""
    seen = set()
    for number, _ in pairs:
        if number in seen:
            raise ValueError(f"Pillar number {number} specified more than once in input.")
        seen.add(number)


EMPTY_SEAT_RADIUS = 22
EMPTY_SEAT_OUTLINE = "#999999"
EMPTY_SEAT_NUMBER_COLOR = "#555555"
FILLED_SEAT_NUMBER_COLOR = "#FFFFFF"


def draw_pillar(draw, center_xy, color_code, number, font):
    """Draws a colored circle (pillar) with its seat NUMBER shown on
    top (not the R/G letter - color alone indicates Red/Green)."""
    x, y = center_xy
    bbox = [x - PILLAR_RADIUS, y - PILLAR_RADIUS, x + PILLAR_RADIUS, y + PILLAR_RADIUS]
    draw.ellipse(bbox, fill=COLORS[color_code], outline="black", width=3)

    text = str(number)
    tb = draw.textbbox((0, 0), text, font=font)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    draw.text((x - tw / 2, y - th / 2 - 2), text, fill=FILLED_SEAT_NUMBER_COLOR, font=font)


def draw_empty_seat(draw, center_xy, number, font):
    """Draws an unfilled pillar seat position, labeled with its
    number, so every one of the 21 seats is always visible on the
    field regardless of whether a pillar is placed there."""
    x, y = center_xy
    bbox = [x - EMPTY_SEAT_RADIUS, y - EMPTY_SEAT_RADIUS, x + EMPTY_SEAT_RADIUS, y + EMPTY_SEAT_RADIUS]
    draw.ellipse(bbox, fill="#FFFFFF", outline=EMPTY_SEAT_OUTLINE, width=2)

    text = str(number)
    tb = draw.textbbox((0, 0), text, font=font)
    tw, th = tb[2] - tb[0], tb[3] - tb[1]
    draw.text((x - tw / 2, y - th / 2 - 2), text, fill=EMPTY_SEAT_NUMBER_COLOR, font=font)


def draw_parking_lot(draw, section="Bottom"):
    """Draws the parking lot boundary lines (pink) and returns the
    left/right line coordinates for label placement."""
    coords = PARKING_LOT_COORDS[section]
    draw.line(coords[0], fill=COLORS["Parking"], width=12)
    draw.line(coords[1], fill=COLORS["Parking"], width=12)
    return coords


def draw_title(image, draw, title, subtitle):
    bar_height = 120
    overlay = Image.new('RGBA', (image.width, bar_height), (40, 40, 40, 220))
    image.paste(overlay, (0, 0), overlay)

    try:
        title_font = ImageFont.truetype("arial.ttf", 46)
        subtitle_font = ImageFont.truetype("arial.ttf", 28)
    except Exception:
        title_font = ImageFont.load_default()
        subtitle_font = title_font

    draw.text((30, 10), title, fill=COLORS["Title"], font=title_font)
    draw.text((30, 68), subtitle, fill="#CCCCCC", font=subtitle_font)


def generate_field_from_pillar_list(pillar_input, output_filename=None,
                                     title="Custom Pillar Configuration",
                                     subtitle=None, save=True):
    """
    Renders a playfield image with Red/Green pillars placed at the
    positions specified in `pillar_input`.

    Args:
        pillar_input: str like "5 G 6 G 10 G 12 G 16 R 18 G 2 R 3 R",
                       or a list of (number, color) tuples / flat tokens.
        output_filename: filename (not full path) to save under
                          OUTPUT_DIR. Auto-generated if not provided.
        title: title bar text.
        subtitle: subtitle bar text; auto-generated from the pillar
                  list if not provided.
        save: if True, saves the image to OUTPUT_DIR and returns the
              path; if False, returns the PIL Image object instead.

    Returns:
        The output file path (str) if save=True, otherwise a PIL Image.
    """
    if isinstance(pillar_input, str):
        pairs = parse_pillar_input(pillar_input)
    else:
        pairs = list(pillar_input)  # assume already (number, color) tuples

    if not os.path.exists(BASE_IMAGE_PATH):
        raise FileNotFoundError(f"Base image not found at: {BASE_IMAGE_PATH}")

    img = Image.open(BASE_IMAGE_PATH).convert("RGBA")
    draw = ImageDraw.Draw(img)

    try:
        font = ImageFont.truetype("arial.ttf", 24)
    except Exception:
        font = ImageFont.load_default()

    # Draw parking lot (always Bottom section per this field orientation)
    draw_parking_lot(draw, "Bottom")

    # Always draw all 21 seat positions first (numbered, unfilled),
    # then overlay the specified pillars on top of their seats.
    occupied_numbers = {number for number, _ in pairs}
    try:
        seat_font = ImageFont.truetype("arial.ttf", 20)
    except Exception:
        seat_font = font

    for number in range(1, 22):
        if number in occupied_numbers:
            continue
        xy = get_pixel_coords(number)
        draw_empty_seat(draw, xy, number, seat_font)

    for number, color_code in pairs:
        xy = get_pixel_coords(number)
        draw_pillar(draw, xy, color_code, number, font)

    # Title / subtitle / info
    if subtitle is None:
        desc = ", ".join(f"{n}{c}" for n, c in pairs)
        subtitle = f"Pillars: {desc}  |  Total: {len(pairs)}"
    draw_title(img, draw, title, subtitle)

    if not save:
        return img

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    if output_filename is None:
        stamp = "_".join(f"{n}{c}" for n, c in pairs)
        output_filename = f"config_{stamp}.png"
    output_path = os.path.join(OUTPUT_DIR, output_filename)
    img.save(output_path)
    return output_path


def parse_config_file(config_file_path):
    """
    Parses an input file containing multiple named pillar configurations,
    one per line, in the format:

        M1 = 5G 6G 10G 12G 16R 18G 2R 3R
        M2 = 5G 7R 10R 15G 19R 1G 3R

    Lines starting with '#' or blank lines are ignored.

    Returns:
        A list of (config_name, pillar_string) tuples, in file order.
    """
    if not os.path.exists(config_file_path):
        raise FileNotFoundError(f"Config file not found: {config_file_path}")

    configs = []
    seen_names = set()

    with open(config_file_path, "r", encoding="utf-8") as f:
        for line_num, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" not in line:
                raise ValueError(
                    f"Line {line_num} in {config_file_path} is malformed "
                    f"(missing '='): {raw_line!r}"
                )
            name_part, pillar_part = line.split("=", 1)
            name = name_part.strip()
            pillar_string = pillar_part.strip()

            if not name:
                raise ValueError(f"Line {line_num}: missing configuration name.")
            if not pillar_string:
                raise ValueError(f"Line {line_num}: missing pillar data for '{name}'.")
            if name in seen_names:
                raise ValueError(f"Duplicate configuration name '{name}' found (line {line_num}).")

            seen_names.add(name)
            configs.append((name, pillar_string))

    return configs


def generate_all_configs_from_file(config_file_path=None, output_dir=None,
                                     image_ext="jpeg", verbose=True):
    """
    Reads all named pillar configurations from a config file and
    generates one image per configuration, named "<ConfigName>.<ext>"
    (e.g. M1.jpeg, M2.jpeg, ...).

    Args:
        config_file_path: path to the input config file. Defaults to
                           "pillar_configs_input.txt" next to this script.
        output_dir: directory to save generated images. Defaults to
                    OUTPUT_DIR ("pillar_configs" folder under test/).
        image_ext: file extension/format to save as (e.g. "jpeg", "png").
        verbose: if True, prints progress for each configuration.

    Returns:
        A list of (config_name, output_path, error_or_None) tuples -
        error_or_None is a string describing the problem if that
        specific configuration failed to generate, else None.
    """
    if config_file_path is None:
        config_file_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "input_config", "pillar_configs_input.txt"
        )
    if output_dir is None:
        output_dir = OUTPUT_DIR

    os.makedirs(output_dir, exist_ok=True)

    configs = parse_config_file(config_file_path)
    results = []

    for name, pillar_string in configs:
        output_filename = f"{name}.{image_ext}"
        output_path = os.path.join(output_dir, output_filename)
        try:
            pairs = parse_pillar_input(pillar_string)
            img = generate_field_from_pillar_list(
                pairs,
                title=f"Configuration: {name}",
                save=False,
            )
            # Convert to RGB before saving as JPEG (JPEG doesn't support alpha)
            if image_ext.lower() in ("jpg", "jpeg"):
                img = img.convert("RGB")
            img.save(output_path)
            results.append((name, output_path, None))
            if verbose:
                print(f"[OK]   {name}: {pillar_string}  ->  {output_path}")
        except ValueError as e:
            results.append((name, output_path, str(e)))
            if verbose:
                print(f"[FAIL] {name}: {pillar_string}  ->  ERROR: {e}")

    if verbose:
        ok_count = sum(1 for _, _, err in results if err is None)
        print(f"\nDone: {ok_count}/{len(results)} configurations generated successfully.")

    return results


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print('  Single config:  python generate_field_from_pillars.py "5 G 6 G 10 G 12 G 16 R 18 G 2 R 3 R"')
        print("  Batch from file: python generate_field_from_pillars.py --batch [config_file.txt]")
        sys.exit(1)

    if sys.argv[1] == "--batch":
        config_file = sys.argv[2] if len(sys.argv) > 2 else None
        generate_all_configs_from_file(config_file_path=config_file)
        return

    pillar_string = " ".join(sys.argv[1:])
    try:
        output_path = generate_field_from_pillar_list(pillar_string)
        print(f"Generated: {output_path}")
    except ValueError as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
