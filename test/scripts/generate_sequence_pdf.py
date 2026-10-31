"""
WRO FE 2026 - Pillar Configuration Sequence PDF Generator
=============================================================
Generates a landscape PDF where each page shows TWO consecutive
field configurations side by side (e.g. M1 & M2 on page 1, M3 & M4
on page 2, ...), along with the transition instructions:
  - the transition FROM the left config TO the right config (shown
    between the two images on the same page), and
  - the transition FROM the right config TO the NEXT config in the
    sequence (shown below, as a preview of what page will follow).

Uses the same movement-cost logic as optimize_sequence.py so the
transition descriptions are always consistent with the actual
optimized run order.
"""

import os
from reportlab.lib.pagesizes import landscape, A4
from reportlab.lib.units import cm
from reportlab.pdfgen import canvas
from reportlab.lib.utils import ImageReader

from generate_field_from_pillars import parse_config_file
from optimize_sequence import config_to_seat_map, describe_transition

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_DIR = os.path.join(BASE_DIR, "..")
CONFIG_FILE = os.path.join(TEST_DIR, "input_config", "pillar_configs_input.txt")
IMAGE_DIR = os.path.join(TEST_DIR, "pillar_configs")
OUTPUT_PDF = os.path.join(TEST_DIR, "master_test_layouts", "phase3", "pillar_sequence_transitions.pdf")


def format_transition_line(name_a, name_b, seat_map_a, seat_map_b):
    """Builds a single concise line describing the pillar movements
    needed to go from config name_a to config name_b."""
    added, removed, swapped, _ = describe_transition(name_a, name_b, seat_map_a, seat_map_b)
    parts = []
    if added:
        parts.append("Place " + ", ".join(f"{s}{c}" for s, c in added))
    if removed:
        parts.append("Remove " + ", ".join(f"{s}({c})" for s, c in removed))
    if swapped:
        parts.append("Swap " + ", ".join(f"{s}({c1}->{c2})" for s, c1, c2 in swapped))
    if not parts:
        return f"{name_a} -> {name_b}: No changes needed."
    total = len(added) + len(removed) + len(swapped)
    return f"{name_a} -> {name_b} ({total} moves): " + "; ".join(parts)


def generate_pdf(config_file=CONFIG_FILE, image_dir=IMAGE_DIR, output_pdf=OUTPUT_PDF):
    configs = parse_config_file(config_file)  # ordered list of (name, pillar_string)
    names = [name for name, _ in configs]
    pillar_strings = dict(configs)
    seat_maps = {name: config_to_seat_map(s) for name, s in configs}

    page_w, page_h = landscape(A4)
    c = canvas.Canvas(output_pdf, pagesize=(page_w, page_h))

    margin = 1.2 * cm
    title_h = 1.0 * cm
    caption_h = 0.6 * cm
    transition_h = 1.6 * cm
    next_preview_h = 0.8 * cm

    available_w = page_w - 2 * margin
    img_w = (available_w - 1.0 * cm) / 2  # gap between the two images
    img_h = img_w  # images are square

    # Recompute so the whole block fits vertically
    content_h = title_h + img_h + caption_h + transition_h + next_preview_h
    if content_h > page_h - 2 * margin:
        scale = (page_h - 2 * margin) / content_h
        img_w *= scale
        img_h *= scale

    # Pair up configs: (M1,M2), (M3,M4), (M5,M6), (M7, None) if odd count
    pairs_of_names = [names[i:i + 2] for i in range(0, len(names), 2)]

    for page_idx, pair in enumerate(pairs_of_names):
        left_name = pair[0]
        right_name = pair[1] if len(pair) > 1 else None

        y_top = page_h - margin

        # --- Page title ---
        c.setFont("Helvetica-Bold", 16)
        page_title = f"{left_name}" + (f"  &  {right_name}" if right_name else "")
        c.drawCentredString(page_w / 2, y_top - 0.2 * cm, f"Field Setup: {page_title}")

        # --- Image positions ---
        img_y = y_top - title_h - img_h
        left_x = margin
        right_x = margin + img_w + 1.0 * cm

        # Left image
        left_path = os.path.join(image_dir, f"{left_name}.jpeg")
        if os.path.exists(left_path):
            c.drawImage(ImageReader(left_path), left_x, img_y, width=img_w, height=img_h,
                        preserveAspectRatio=True, anchor='c')
        c.setFont("Helvetica-Bold", 11)
        c.drawCentredString(left_x + img_w / 2, img_y - 0.4 * cm, left_name)
        c.setFont("Helvetica", 8)
        c.drawCentredString(left_x + img_w / 2, img_y - 0.75 * cm, pillar_strings[left_name])

        # Right image (if present)
        if right_name:
            right_path = os.path.join(image_dir, f"{right_name}.jpeg")
            if os.path.exists(right_path):
                c.drawImage(ImageReader(right_path), right_x, img_y, width=img_w, height=img_h,
                            preserveAspectRatio=True, anchor='c')
            c.setFont("Helvetica-Bold", 11)
            c.drawCentredString(right_x + img_w / 2, img_y - 0.4 * cm, right_name)
            c.setFont("Helvetica", 8)
            c.drawCentredString(right_x + img_w / 2, img_y - 0.75 * cm, pillar_strings[right_name])

        # --- Transition line: left -> right ---
        trans_y = img_y - caption_h - 0.5 * cm
        c.setFont("Helvetica-Bold", 10)
        if right_name:
            line1 = format_transition_line(left_name, right_name, seat_maps[left_name], seat_maps[right_name])
        else:
            line1 = f"{left_name}: final configuration in the sequence."
        c.setFillColorRGB(0.55, 0.0, 0.55)  # dark magenta, matches parking line color theme
        c.drawCentredString(page_w / 2, trans_y, line1)

        # --- Next preview line: right -> next config (if any) ---
        next_idx = names.index(right_name) + 1 if right_name else names.index(left_name) + 1
        if next_idx < len(names):
            next_name = names[next_idx]
            from_name = right_name if right_name else left_name
            line2 = format_transition_line(from_name, next_name, seat_maps[from_name], seat_maps[next_name])
            c.setFont("Helvetica-Oblique", 9)
            c.setFillColorRGB(0.35, 0.35, 0.35)
            c.drawCentredString(page_w / 2, trans_y - 0.55 * cm,
                                 f"(Next up: {line2})")

        c.setFillColorRGB(0, 0, 0)
        c.showPage()

    c.save()
    return output_pdf


if __name__ == "__main__":
    path = generate_pdf()
    print(f"Generated PDF: {path}")
