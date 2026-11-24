"""
WRO FE 2026 - Print PDF Maker (no optimisation, just layout images -> PDF)
==========================================================================
Combines already-generated layout images (test/pillar_configs/<Name>.jpeg)
into one printable PDF: N images per page (default 2, side by side),
landscape A4, each image scaled as large as the page allows.
Page format follows generate_sequence_pdf.py (page title "Field Setup: A & B",
layout name + pillar list under each image) but WITHOUT the transition /
optimisation lines, and it does not import optimize_sequence.

Layout names are the names used in input_config/pillar_configs_input.txt
(e.g. S1, C8a, M3). Settings and named "jobs" live in
input_config/print_jobs.json so they can be re-used later.

USAGE (run from test/scripts)
-----------------------------
    python make_print_pdf.py --list                  # show saved jobs
    python make_print_pdf.py --job phase2_all        # run a saved job
    python make_print_pdf.py --layouts S1 S2 C1-C24 C8a --out my_print.pdf
    python make_print_pdf.py --job phase2_all --per-page 1 --portrait

A name like "C1-C24" expands to C1, C2, ... C24. Order given = order printed.
Missing images are reported and skipped (the PDF is still produced).
"""

import argparse
import json
import os
import re
import sys
import tempfile

from PIL import Image
from reportlab.lib import pagesizes
from reportlab.lib.units import cm, mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

# Reuse the config-file parser (gives the pillar list shown under each image)
from generate_field_from_pillars import parse_config_file, generate_field_from_pillar_list

BLANK_NAME = "BLANK"
BLANK_CAPTION = "(empty field - draw a new combination at run time)"

# Page furniture sizes, same values as generate_sequence_pdf.py
TITLE_H = 1.0 * cm
CAPTION_H = 0.9 * cm  # room for name + pillar list under each image

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CONFIG = os.path.join(BASE_DIR, "..", "input_config", "print_jobs.json")

DEFAULT_SETTINGS = {
    "image_dir": "../pillar_configs",
    "image_ext": "jpeg",
    "config_file": "pillar_configs_input.txt",
    "show_title": True,
    "show_caption": True,
    "pad_blank": True,
    "output_dir": "../print_pdfs",
    "page_size": "A4",
    "orientation": "landscape",
    "per_page": 2,
    "margin_mm": 5,
    "gap_mm": 4,
}


def load_config(path):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    settings = dict(DEFAULT_SETTINGS)
    settings.update(data.get("settings", {}))
    return settings, data.get("jobs", {})


def expand_names(tokens):
    """['S1', 'C1-C3', 'C8a'] -> ['S1', 'C1', 'C2', 'C3', 'C8a']"""
    names = []
    for tok in tokens:
        tok = tok.strip()
        m = re.fullmatch(r"([A-Za-z_]*)(\d+)\s*-\s*([A-Za-z_]*)(\d+)", tok)
        if m and (m.group(3) in ("", m.group(1))):
            prefix, lo, hi = m.group(1), int(m.group(2)), int(m.group(4))
            step = 1 if hi >= lo else -1
            names.extend(f"{prefix}{n}" for n in range(lo, hi + step, step))
        else:
            names.append(tok)
    return names


def resolve_path(p):
    return p if os.path.isabs(p) else os.path.normpath(os.path.join(os.path.dirname(DEFAULT_CONFIG), p))


def find_image(image_dir, name, ext):
    """Exact (case-insensitive) match on <name>.<ext>, then any common extension."""
    exts = [ext] + [e for e in ("jpeg", "jpg", "png") if e != ext]
    try:
        files = {f.lower(): f for f in os.listdir(image_dir)}
    except FileNotFoundError:
        return None
    for e in exts:
        hit = files.get(f"{name}.{e}".lower())
        if hit:
            return os.path.join(image_dir, hit)
    return None


def make_blank_field_image():
    """Renders the empty numbered field (all 21 seats, no pillars) to a temp
    JPEG, using the same renderer as the generated layouts. Returns its path."""
    img = generate_field_from_pillar_list(
        [], title="Configuration: BLANK (new combination)",
        subtitle="Pillars: ______________________________  |  Total: ___", save=False)
    fd, path = tempfile.mkstemp(suffix=".jpeg", prefix="blank_field_")
    os.close(fd)
    img.convert("RGB").save(path, quality=95)
    return path


def build_pdf(names, output_pdf, s):
    image_dir = resolve_path(s["image_dir"])
    per_page = int(s["per_page"])
    page_w, page_h = getattr(pagesizes, s["page_size"].upper())
    if s["orientation"].lower() == "landscape":
        page_w, page_h = max(page_w, page_h), min(page_w, page_h)
    else:
        page_w, page_h = min(page_w, page_h), max(page_w, page_h)

    margin, gap = s["margin_mm"] * mm, s["gap_mm"] * mm
    # Same page furniture as generate_sequence_pdf.py (minus the transition lines):
    # a page title on top and "name + pillar list" caption under each image.
    title_h = TITLE_H if s.get("show_title", True) else 0
    caption_h = CAPTION_H if s.get("show_caption", True) else 0

    # Landscape: images side by side (columns). Portrait: stacked (rows).
    side_by_side = page_w >= page_h
    avail_w, avail_h = page_w - 2 * margin, page_h - 2 * margin - title_h
    if side_by_side:
        cell_w, cell_h = (avail_w - gap * (per_page - 1)) / per_page, avail_h
    else:
        cell_w, cell_h = avail_w, (avail_h - gap * (per_page - 1)) / per_page
    img_box_h = cell_h - caption_h

    try:
        pillar_strings = dict(parse_config_file(resolve_path(s["config_file"])))
    except (FileNotFoundError, ValueError) as e:
        print(f"  NOTE - pillar lists not available for captions: {e}")
        pillar_strings = {}

    found, missing = [], []
    for n in names:
        p = find_image(image_dir, n, s["image_ext"])
        (found if p else missing).append((n, p) if p else n)

    # Odd count -> pad the last page with an empty numbered field (no pillars)
    # so a new combination can be drawn on it at run time.
    blank_tmp = None
    pad = (-len(found)) % per_page if s.get("pad_blank", True) and found else 0
    if pad:
        blank_tmp = make_blank_field_image()
        for k in range(pad):
            found.append((BLANK_NAME if pad == 1 else f"{BLANK_NAME} {k + 1}", blank_tmp))
            pillar_strings[found[-1][0]] = BLANK_CAPTION

    os.makedirs(os.path.dirname(output_pdf), exist_ok=True)
    c = canvas.Canvas(output_pdf, pagesize=(page_w, page_h))
    c.setTitle("Layouts print")

    for start in range(0, len(found), per_page):
        group = found[start:start + per_page]
        if title_h:
            c.setFont("Helvetica-Bold", 16)
            c.drawCentredString(page_w / 2, page_h - margin - 0.2 * cm,
                                "Field Setup: " + "  &  ".join(n for n, _ in group))
        for slot, (name, path) in enumerate(group):
            with Image.open(path) as im:
                iw, ih = im.size
            scale = min(cell_w / iw, img_box_h / ih)
            w, h = iw * scale, ih * scale
            if side_by_side:
                cell_x = margin + slot * (cell_w + gap)
                cell_top = page_h - margin - title_h
            else:
                cell_x = margin
                cell_top = page_h - margin - title_h - slot * (cell_h + gap)
            x0 = cell_x + (cell_w - w) / 2
            y0 = cell_top - h
            c.drawImage(ImageReader(path), x0, y0, w, h)
            if caption_h:
                c.setFillColorRGB(0, 0, 0)
                c.setFont("Helvetica-Bold", 11)
                c.drawCentredString(cell_x + cell_w / 2, y0 - 0.4 * cm, name)
                if name in pillar_strings:
                    c.setFont("Helvetica", 8)
                    c.drawCentredString(cell_x + cell_w / 2, y0 - 0.75 * cm, pillar_strings[name])
        c.showPage()
    c.save()
    if blank_tmp and os.path.exists(blank_tmp):
        os.remove(blank_tmp)

    pages = -(-len(found) // per_page)
    print(f"Wrote {output_pdf}")
    print(f"  {len(found) - pad} images" + (f" + {pad} blank field(s)" if pad else "") +
          f", {pages} pages, {per_page} per page, "
          f"{s['page_size']} {s['orientation']}, image size {min(cell_w, img_box_h) / mm:.0f} mm")
    if missing:
        print(f"  WARNING - image not found, skipped: {', '.join(missing)}")
    return pages


def main():
    ap = argparse.ArgumentParser(description="Combine layout images into a printable PDF.")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="path to print_jobs.json")
    ap.add_argument("--job", help="name of a saved job in the config file")
    ap.add_argument("--list", action="store_true", help="list saved jobs and exit")
    ap.add_argument("--layouts", nargs="+", help="layout names / ranges, e.g. S1 S2 C1-C24 C8a")
    ap.add_argument("--out", help="output PDF file name or path")
    ap.add_argument("--per-page", type=int, help="images per page (default from config)")
    ap.add_argument("--portrait", action="store_true", help="portrait instead of landscape")
    ap.add_argument("--no-blank", action="store_true", help="do not pad an odd last page with a blank field")
    args = ap.parse_args()

    settings, jobs = load_config(args.config)

    if args.list:
        for jn, j in jobs.items():
            print(f"{jn}: {' '.join(j['layouts'])}  ->  {j.get('output', jn + '.pdf')}")
        return

    if args.job:
        if args.job not in jobs:
            sys.exit(f"Job '{args.job}' not found. Available: {', '.join(jobs) or '(none)'}")
        job = jobs[args.job]
        tokens = job["layouts"]
        out = args.out or job.get("output", f"{args.job}.pdf")
        for k in ("per_page", "orientation", "page_size", "margin_mm", "gap_mm", "image_dir",
                  "image_ext", "config_file", "show_title", "show_caption", "pad_blank"):
            if k in job:
                settings[k] = job[k]
    elif args.layouts:
        tokens = args.layouts
        out = args.out or "layouts_print.pdf"
    else:
        ap.error("give --job NAME or --layouts ... (or --list)")

    if args.per_page:
        settings["per_page"] = args.per_page
    if args.portrait:
        settings["orientation"] = "portrait"
    if args.no_blank:
        settings["pad_blank"] = False

    out_path = out if os.path.isabs(out) or os.path.dirname(out) else os.path.join(resolve_path(settings["output_dir"]), out)
    build_pdf(expand_names(tokens), out_path, settings)


if __name__ == "__main__":
    main()
