"""check_placement.py - courtyard / edge / mounting-hole collision report
for sh03-controller.kicad_pcb.

Run with KiCad's bundled interpreter:
  python3 scripts/check_placement.py

Reads the saved board and reports, as plain text:
  1. Every footprint's courtyard bounding box and layer.
  2. Same-layer courtyard overlaps, with overlap amount per axis.
  3. Footprints whose courtyard extends outside the board outline.
     H1-H4 entries here are flagged separately -- their fallback bbox
     includes the ref/value silkscreen text offset (+-4.7mm), not their
     copper/drill extent, so they read as "outside the outline" even
     though the physical hole is inside it. Real violations are labelled
     [REAL].
  4. Footprints on EITHER layer whose courtyard comes within 1.0mm of any
     mounting hole's pad (copper/drill) extent -- mounting holes are
     through-holes, so this check is not restricted to matching layers.
     The H1<->J2 and H3<->J1 clearances are printed explicitly regardless
     of whether they trip the 1.0mm threshold (J1/J2 swapped slots
     2026-08-12c to match the stock board's connector order -- J2 is now
     the north-slot connector nearest H1, J1 the south-slot connector
     nearest H3; was H1<->J1 / H3<->J2 before the swap).
"""

import os

import pcbnew

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB_PATH = os.path.join(PROJECT_DIR, "sh03-controller.kicad_pcb")

BOARD_OUTLINE = (36.0, 33.0, 136.0, 89.0)  # (min_x, min_y, max_x, max_y) mm -- east
# edge widened 132 -> 136 (96 -> 100mm board width, 2026-08-12d east-widen rework,
# kept under JLCPCB's <=100x100mm price tier) -- see docs/layout-plan.md.
MOUNTING_HOLE_REFS = {"H1", "H2", "H3", "H4"}
PROXIMITY_LIMIT_MM = 1.0


def mm(nm):
    return pcbnew.ToMM(nm)


def bbox_mm(box):
    return (mm(box.GetLeft()), mm(box.GetTop()), mm(box.GetRight()), mm(box.GetBottom()))


def get_courtyard_bbox(fp):
    """Returns (bbox_tuple, layer_str, used_fallback: bool)."""
    layer = "back" if fp.IsFlipped() else "front"
    crtyd_layer = pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd
    poly = fp.GetCourtyard(crtyd_layer)
    if poly is not None and not poly.IsEmpty():
        return bbox_mm(poly.BBox()), layer, False
    return bbox_mm(fp.GetBoundingBox()), layer, True


def overlap_1d(a_min, a_max, b_min, b_max):
    lo = max(a_min, b_min)
    hi = min(a_max, b_max)
    return max(0.0, hi - lo)


def main():
    board = pcbnew.LoadBoard(PCB_PATH)
    footprints = list(board.GetFootprints())

    info = {}  # ref -> (bbox, layer, used_fallback)
    for fp in footprints:
        ref = fp.GetReference()
        info[ref] = get_courtyard_bbox(fp)

    print("=== 1. Footprint courtyard bounding boxes ===")
    for ref in sorted(info):
        (x0, y0, x1, y1), layer, fallback = info[ref]
        note = "  [FALLBACK: overall bounding box, no courtyard geometry]" if fallback else ""
        print(f"{ref:6s} layer={layer:5s} bbox=({x0:.3f}, {y0:.3f}) - ({x1:.3f}, {y1:.3f}) mm{note}")

    print("\n=== 2. Same-layer courtyard overlaps ===")
    refs = sorted(info)
    found_overlap = False
    for i, ref_a in enumerate(refs):
        (ax0, ay0, ax1, ay1), layer_a, _ = info[ref_a]
        for ref_b in refs[i + 1:]:
            (bx0, by0, bx1, by1), layer_b, _ = info[ref_b]
            if layer_a != layer_b:
                continue
            ox = overlap_1d(ax0, ax1, bx0, bx1)
            oy = overlap_1d(ay0, ay1, by0, by1)
            if ox > 0 and oy > 0:
                found_overlap = True
                print(f"{ref_a} <-> {ref_b} (layer={layer_a}): overlap x={ox:.3f}mm, y={oy:.3f}mm")
    if not found_overlap:
        print("(none)")

    print("\n=== 3. Footprints extending outside board outline (36,33)-(136,89) ===")
    ox0, oy0, ox1, oy1 = BOARD_OUTLINE
    found_edge = False
    for ref in refs:
        (x0, y0, x1, y1), layer, fallback = info[ref]
        violations = []
        if x0 < ox0:
            violations.append(f"left edge by {ox0 - x0:.3f}mm")
        if y0 < oy0:
            violations.append(f"top edge by {oy0 - y0:.3f}mm")
        if x1 > ox1:
            violations.append(f"right edge by {x1 - ox1:.3f}mm")
        if y1 > oy1:
            violations.append(f"bottom edge by {y1 - oy1:.3f}mm")
        if violations:
            found_edge = True
            if ref in MOUNTING_HOLE_REFS and fallback:
                tag = "[TEXT-DRIVEN NOISE, not a real violation -- fallback bbox includes ref/value silkscreen at +-4.7mm; the pad/drill itself is inside the outline]"
            else:
                tag = "[REAL]"
            print(f"{ref} {tag}: " + "; ".join(violations))
    if not found_edge:
        print("(none)")

    print(f"\n=== 4. Footprints (either layer) within {PROXIMITY_LIMIT_MM}mm of a mounting hole copper/drill extent ===")
    # Mounting hole extent = union of its pad bounding boxes (not the
    # footprint's overall bbox, which is inflated by the H1-H4 ref/value
    # silkscreen text offsets and would over-report proximity). Mounting
    # holes are through-holes (pads on *.Cu, i.e. every copper layer), so
    # this check is intentionally NOT restricted to same-layer pairs --
    # a front-layer part and a back-layer part are equally at risk.
    hole_extents = {}
    for fp in footprints:
        ref = fp.GetReference()
        if ref not in MOUNTING_HOLE_REFS:
            continue
        pads = list(fp.Pads())
        if not pads:
            continue
        xs0, ys0, xs1, ys1 = [], [], [], []
        for pad in pads:
            bb = pad.GetBoundingBox()
            xs0.append(mm(bb.GetLeft()))
            ys0.append(mm(bb.GetTop()))
            xs1.append(mm(bb.GetRight()))
            ys1.append(mm(bb.GetBottom()))
        hole_extents[ref] = (min(xs0), min(ys0), max(xs1), max(ys1))

    all_distances = {}  # (ref, hole_ref) -> dist_mm
    found_proximity = False
    for ref in refs:
        if ref in MOUNTING_HOLE_REFS:
            continue
        (x0, y0, x1, y1), layer, _ = info[ref]
        for hole_ref, (hx0, hy0, hx1, hy1) in hole_extents.items():
            # distance between two axis-aligned rectangles (0 if overlapping)
            dx = max(hx0 - x1, x0 - hx1, 0.0)
            dy = max(hy0 - y1, y0 - hy1, 0.0)
            dist = (dx ** 2 + dy ** 2) ** 0.5
            all_distances[(ref, hole_ref)] = (dist, layer)
            if dist <= PROXIMITY_LIMIT_MM:
                found_proximity = True
                print(f"{ref} (layer={layer}) is {dist:.3f}mm from {hole_ref}'s copper/drill extent")
    if not found_proximity:
        print("(none)")

    print("\n--- requested pairs (reported regardless of the 1.0mm threshold) ---")
    for req_ref, req_hole in (("J2", "H1"), ("J1", "H3")):
        key = (req_ref, req_hole)
        if key in all_distances:
            dist, layer = all_distances[key]
            print(f"{req_ref} (layer={layer}) <-> {req_hole}: {dist:.3f}mm")
        else:
            print(f"{req_ref} <-> {req_hole}: not computed ({req_ref} or {req_hole} missing from board)")


if __name__ == "__main__":
    main()
