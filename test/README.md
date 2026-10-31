# Field Testing (`test/`)

This folder holds everything behind our obstacle field testing: the layouts we ran, the handwritten and typed run logs, and the scripts that generate the layouts and the competition-day run-book. The story of why we tested this way, and what we found, is in **Section 4.9.2 – Obstacle Challenge Testing** of the main [README](../README.md#492-obstacle-challenge-testing).

## Folder map

| Folder | What is in it |
|---|---|
| [`scripts/`](scripts/) | The five Python scripts (see below). |
| [`input_config/`](input_config/) | Inputs to the scripts: the base playfield image, the M1–M7 layout list, and the seat-numbering check image. |
| [`master_test_layouts/phase1/`](master_test_layouts/phase1/) | Phase 1 layout diagrams (MS1–MS6, clockwise and anti-clockwise, plus the tough-layout examples) and the printed run-sheet template [`WRO_FE_2026_Master_Test_Tracker_v2.pdf`](master_test_layouts/phase1/WRO_FE_2026_Master_Test_Tracker_v2.pdf). |
| [`master_test_layouts/phase3/`](master_test_layouts/phase3/) | The competition run-book [`pillar_sequence_transitions.pdf`](master_test_layouts/phase3/pillar_sequence_transitions.pdf) and a preview of its first page. |
| [`pillar_configs/`](pillar_configs/) | Field diagrams for the seven Phase 3 layouts, `M1.jpeg` to `M7.jpeg`. |
| [`field_logs/`](field_logs/) | Phase 2 and Phase 3 run records: scanned handwritten sheets and a typed copy, [`coverage_matrix.md`](field_logs/coverage_matrix.md). |

### `input_config/`

| File | Purpose |
|---|---|
| `WRO-2025_FutureEngineers_Playfield.jpg` | Base playfield image that all layout diagrams are drawn on. |
| `pillar_configs_input.txt` | The seven M1–M7 layouts, one per line (for example `M5 = 5G 6G 10G 12G 16R 18G 2R 3R`). This is the one place to edit the test set. |
| `diagnostic_pillar_numbering.png` | Output of `markup_pillar_numbers.py`: the seat numbers drawn on the playfield, used to check our numbering by eye. |

### `field_logs/`

| File | Purpose |
|---|---|
| `field_seat_numbering.jpeg` | The printed field with all 21 pillar seats numbered by hand. |
| `coverage_design_C1-C24.jpeg` | How the 24 Phase 2 combinations were worked out. |
| `Test_log_phase2_pg1.jpeg` | Full layout, run times and merge into M1–M7. |
| `Test_log_phase2_pg2.jpeg` | Defects from the first testing round and the re-verification pass. |
| `coverage_matrix.md` | Typed layouts, run logs and issues for Phases 1–3. The scans are the original record. |

## Seat numbering

Pillars are placed by seat number (1–21) plus a colour, written as `<seat><G|R>`, for example `5G 6G 10G 12G 16R 18G 2R 3R`.

- **Bottom (parking side):** seats 3, 4, 5, left to right. There is no outer row on this side.
- **Right:** inner 6, 8, 10; outer 7, 9, 11.
- **Top:** inner 12, 14, 16; outer 13, 15, 17.
- **Left:** inner 18, 20, 2; outer 19, 21, 1.

Inner is towards the inner track wall; outer is towards the outer boundary wall.

## Scripts (`scripts/`)

| Script | Phase | What it does | Reads | Writes |
|---|---|---|---|---|
| `generate_tough_configs.py` | 1 | Builds the tough layouts from the official WRO layout cards. It picks cards where the pillar colour and side force a tight wall gap. | playfield image | `master_test_layouts/phase1/` |
| `markup_pillar_numbers.py` | 2 | One-time check. Draws seats 1–21 on the playfield so our numbering could be verified by eye. | playfield image | `input_config/diagnostic_pillar_numbering.png` |
| `generate_field_from_pillars.py` | 2–3 | Draws a field diagram from a pillar string. With `--batch`, draws every layout in `pillar_configs_input.txt`. | playfield image, `pillar_configs_input.txt` | `pillar_configs/` |
| `optimize_sequence.py` | 3 | Counts the pillar moves between every pair of layouts and tries all 5040 orderings of M1–M7. Prints the run order with the fewest total moves (34 for the current set). | `pillar_configs_input.txt` | console only |
| `generate_sequence_pdf.py` | 3 | Builds the run-book: two layouts per page with the exact moves between them. | `pillar_configs_input.txt`, `pillar_configs/` | `master_test_layouts/phase3/pillar_sequence_transitions.pdf` |

## How to run

Requires Python 3 with `pillow` and `reportlab` (`pip install pillow reportlab`). Run from inside `test/scripts/`:

```bash
cd test/scripts

python generate_tough_configs.py                    # Phase 1 tough layouts
python generate_field_from_pillars.py --batch       # M1–M7 diagrams from input_config/pillar_configs_input.txt
python optimize_sequence.py                         # prints the lowest-movement run order
python generate_sequence_pdf.py                     # builds the Phase 3 run-book PDF
```

To try a single layout, pass the pillar string directly:

```bash
python generate_field_from_pillars.py "5G 6G 10G 12G 16R 18G 2R 3R"
```

To change the test set, edit `input_config/pillar_configs_input.txt`, then re-run the last three commands in the order shown.
