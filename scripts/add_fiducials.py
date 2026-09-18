"""add_fiducials.py - three B.Cu fiducial markers for back-side pick-and-place
on sh03-controller.kicad_pcb.

Run with KiCad's bundled interpreter:
  python3 scripts/add_fiducials.py

Context: J1/J2 (1.25mm-pitch SMD connectors, both on B.Cu) are machine-
placed by a pick-and-place assembly service; everything on F.Cu is hand-
soldered. The service needs fiducials to register the board.

Pipeline position: LAST stage, after add_ground_pour.py (place_footprints.py
-> route_board.py -> add_ground_pour.py -> add_fiducials.py). It has to run
after the pour and the perimeter stitching vias exist, because two of its
five obstacle categories -- "the GND pour" and "the perimeter stitching
vias" -- are only real, checkable geometry once add_ground_pour.py has run.

Fiducial spec: 1.0mm copper dot (SMD pad, circle, B.Cu only, no drill, no
net), 2.0mm solder-mask opening. Each pad carries a 2.0mm local clearance
override. That override is what actually makes the required "clear of GND
pour" gap real: a plain search across the already-filled board would never
find a naturally-occurring 2mm-clear spot, because the pour already covers
essentially the entire back face down to its own ~0.15mm clearance. The
override forces the zone filler to retreat around the pad; this script
re-fills the B.Cu GND zone after adding the pads so the saved board
reflects that retreat, then empirically measures the resulting gap with
ZONE.HitTestFilledArea() -- a real hit-test against the actual filled
polygon, not a theoretical value read back from the override setting.

The other four obstacle categories (B.Cu pads -- including J1/J2, U1's
through-hole pads, J3, and the H1-H4 mounting holes -- B.Cu trace segments,
vias, and the board edge) are all fixed geometry that already exists before
this script runs, so their clearance is found by direct grid search against
the loaded board, not simulated.

Candidate search: each fiducial's position is the point of maximum measured
clearance within a hand-picked search box (SEARCH_BOXES below), chosen from
inspection of the routed board -- J1/J2's pad column plus H1/H3 make the
whole left third of the board unusable for a 2.5mm-radius keepout except a
narrow strip between J1's bottom and the H3/J2 corner cluster; the
right two-thirds has open pockets top-right and bottom-right once J3, U1's
pad rows, and the GND stitching vias are avoided. The three land in an
asymmetric arrangement by construction: reflecting any one of them through
the board's centre point lands nowhere near either of the other two, so a
180-degree rotation of the board is detectable by the P&P vision system.

Idempotent: removes any existing FID1-FID3 footprints (by reference) before
recomputing and re-adding them, so re-running doesn't stack duplicates.
Note this means a standalone place_footprints.py run will silently delete
them again (they aren't in its PRESERVED_REFS -- they don't need to be,
since this script is their generator, the same relationship
add_ground_pour.py has with GND zones and stitching vias, not the relationship
place_footprints.py has with the physically-fixed H1-H4 mounting holes).

Determinism: footprint/pad UUIDs are derived from stable keys, same pattern
as the other three pipeline scripts (see place_footprints.py's docstring).
The grid search itself is a pure function of fixed board geometry with a
fixed grid pitch and a total tie-break (max clearance, then lowest x, then
lowest y), so it produces the same three positions on every run against the
same upstream board state.

KiCad 10.0.5 gotcha found writing this: PCB_VIA.GetWidth() with no layer
argument hits a false-assert deep in pcb_track.cpp that manifests as an
indefinite hang under a non-interactive session (no console to dismiss the
assert dialog) -- not a crash, not a timeout, just silence. Always pass an
explicit layer: GetWidth(pcbnew.F_Cu).
"""

import math
import os
import uuid

import pcbnew

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB_PATH = os.path.join(PROJECT_DIR, "sh03-controller.kicad_pcb")

UUID_NAMESPACE = uuid.UUID("25a1abba-b61d-5666-8453-b2e72c16e5ab")
# Stable namespace for deterministic object identifiers.


def stable_uuid(key):
    return str(uuid.uuid5(UUID_NAMESPACE, key))


def set_uuid(item, key):
    """See place_footprints.py's set_uuid for why .Clone() is the write path."""
    item.m_Uuid.Clone(pcbnew.KIID(stable_uuid(key)))


def fmm(v):
    return pcbnew.FromMM(v)


def mm(nm):
    return pcbnew.ToMM(nm)


BOARD_OUTLINE = (36.0, 33.0, 136.0, 89.0)  # (min_x, min_y, max_x, max_y) mm -- east
# edge widened 132 -> 136 (96 -> 100mm board width, 2026-08-12d east-widen rework,
# kept under JLCPCB's <=100x100mm price tier) -- see docs/layout-plan.md.

FID_REFS = ["FID1", "FID2", "FID3"]
FID_COPPER_DIA_MM = 1.0
FID_MASK_OPENING_MM = 2.0
FID_CLEARANCE_MM = 2.0
FID_RADIUS_MM = FID_COPPER_DIA_MM / 2.0

# Search boxes (min_x, min_y, max_x, max_y) mm, one per fiducial -- see the
# module docstring for why these three regions and not, say, three corners.
SEARCH_BOXES = {
    "FID1": (115.0, 35.5, 129.5, 46.0),   # top-right, clear of H2 and its vias
    "FID2": (110.0, 80.0, 129.5, 86.5),   # bottom-right, clear of J3
    "FID3": (38.5, 40.0, 43.5, 88.0),     # left strip between J1/J2 and the edge
}
GRID_PITCH_MM = 0.25

# Empirical GND-pour clearance measurement (post-refill): radial sweep
# around the fiducial's pad edge, reporting the smallest radius at which
# ZONE.HitTestFilledArea() goes solid.
POUR_ANGLE_STEP_DEG = 15
POUR_RADIUS_STEP_MM = 0.1
POUR_RADIUS_MAX_MM = 8.0


# ---------------------------------------------------------------------------
# geometry helpers
# ---------------------------------------------------------------------------

def dist_point_to_rect(px, py, x0, y0, x1, y1):
    dx = max(x0 - px, px - x1, 0.0)
    dy = max(y0 - py, py - y1, 0.0)
    return math.hypot(dx, dy)


def dist_point_to_seg(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    length2 = dx * dx + dy * dy
    if length2 == 0:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / length2))
    projx, projy = x0 + t * dx, y0 + t * dy
    return math.hypot(px - projx, py - projy)


def collect_bcu_pad_rects(board):
    """Bounding box (mm) of every pad present on B.Cu, across every
    footprint on the board -- walked in explicit (refdes, pad number)
    sorted order, matching the rest of the pipeline's determinism
    convention (see route_board.py's module docstring)."""
    rects = []
    for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
        for pad in sorted(fp.Pads(), key=lambda p: p.GetNumber()):
            if not pad.GetLayerSet().Contains(pcbnew.B_Cu):
                continue
            bb = pad.GetBoundingBox()
            rects.append((mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom())))
    return rects


def collect_bcu_track_segs(board):
    segs = []
    for t in board.Tracks():
        if t.GetClass() != "PCB_TRACK":
            continue
        if t.GetLayer() != pcbnew.B_Cu:
            continue
        s, e = t.GetStart(), t.GetEnd()
        segs.append((mm(s.x), mm(s.y), mm(e.x), mm(e.y), mm(t.GetWidth())))
    return segs


def collect_vias(board):
    """Every via on the board (through vias sit on every copper layer,
    B.Cu included). GetWidth() needs an explicit layer -- see the module
    docstring's KiCad 10.0.5 gotcha note."""
    out = []
    for t in board.Tracks():
        if t.GetClass() != "PCB_VIA":
            continue
        p = t.GetPosition()
        out.append((mm(p.x), mm(p.y), mm(t.GetWidth(pcbnew.F_Cu))))
    return out


def min_fixed_clearance(cx, cy, pad_rects, track_segs, via_list):
    """Edge-to-edge clearance in mm from a FID_RADIUS_MM copper dot centred
    at (cx, cy) to the nearest fixed B.Cu obstacle: pad, track, via, or
    board edge. Does NOT include the GND pour -- that's checked separately,
    after the pads exist and the zone has been refilled (see
    measure_pour_clearance)."""
    best = None
    for (x0, y0, x1, y1) in pad_rects:
        d = dist_point_to_rect(cx, cy, x0, y0, x1, y1) - FID_RADIUS_MM
        if best is None or d < best:
            best = d
    for (x0, y0, x1, y1, w) in track_segs:
        d = dist_point_to_seg(cx, cy, x0, y0, x1, y1) - w / 2.0 - FID_RADIUS_MM
        if best is None or d < best:
            best = d
    for (vx, vy, w) in via_list:
        d = math.hypot(cx - vx, cy - vy) - w / 2.0 - FID_RADIUS_MM
        if best is None or d < best:
            best = d
    ox0, oy0, ox1, oy1 = BOARD_OUTLINE
    d_edge = min(cx - ox0, ox1 - cx, cy - oy0, oy1 - cy) - FID_RADIUS_MM
    if best is None or d_edge < best:
        best = d_edge
    return best


def find_best_candidate(box, pad_rects, track_segs, via_list):
    """Grid search: point of maximum fixed-obstacle clearance within `box`.
    Total tie-break (clearance desc, x asc, y asc) keeps this deterministic
    regardless of float-accumulation order."""
    x0, y0, x1, y1 = box
    best = None  # (clearance, x, y)
    nx = round((x1 - x0) / GRID_PITCH_MM)
    ny = round((y1 - y0) / GRID_PITCH_MM)
    for ix in range(nx + 1):
        x = round(x0 + ix * GRID_PITCH_MM, 2)
        for iy in range(ny + 1):
            y = round(y0 + iy * GRID_PITCH_MM, 2)
            c = min_fixed_clearance(x, y, pad_rects, track_segs, via_list)
            key = (c, x, y)
            if best is None or (key[0] > best[0]) or (key[0] == best[0] and (key[1], key[2]) < (best[1], best[2])):
                best = key
    return best


# ---------------------------------------------------------------------------
# board mutation
# ---------------------------------------------------------------------------

def remove_existing_fiducials(board, keep_alive):
    """See place_footprints.py's module docstring for why removed FOOTPRINT
    proxies must be kept alive for the rest of the script's run (KiCad
    10.0.5 pcbnew SWIG GC bug)."""
    removed = 0
    for fp in list(board.GetFootprints()):
        if fp.GetReference() in FID_REFS:
            board.Remove(fp)
            keep_alive.append(fp)
            removed += 1
    return removed


def make_fiducial(board, ref, x, y):
    """Built front-side (F.Cu/F.Mask, F.CrtYd) then Flip()'d to the back --
    same convention place_footprints.py uses for J1/J2, so Flip() carries
    every layer (copper, mask, courtyard, silkscreen) to its back-side
    counterpart in one step instead of hand-picking B.* layers upfront and
    getting it half right. A tight courtyard matching the pad is included
    so check_placement.py's courtyard-overlap/edge checks measure the real
    1mm copper dot, not its fallback (silkscreen-text-inflated) bounding
    box -- the same artifact already documented for H1-H4 in
    docs/layout-plan.md. Reference/value text is hidden; an 8-character
    "FIDUCIAL" label would itself be a silkscreen-over-copper problem this
    close to J1/J2 and U1."""
    pos = pcbnew.VECTOR2I(fmm(x), fmm(y))

    fp = pcbnew.FOOTPRINT(board)
    fp.SetReference(ref)
    fp.SetValue("FIDUCIAL")
    fp.SetPosition(pos)
    fp.Reference().SetVisible(False)
    fp.Value().SetVisible(False)

    pad = pcbnew.PAD(fp)
    pad.SetNumber("")
    pad.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
    pad.SetShape(pcbnew.PAD_SHAPE_CIRCLE)
    pad.SetSize(pcbnew.VECTOR2I(fmm(FID_COPPER_DIA_MM), fmm(FID_COPPER_DIA_MM)))
    pad.SetPosition(pos)
    lset = pcbnew.LSET()
    lset.AddLayer(pcbnew.F_Cu)
    lset.AddLayer(pcbnew.F_Mask)
    pad.SetLayerSet(lset)
    mask_margin = (FID_MASK_OPENING_MM - FID_COPPER_DIA_MM) / 2.0
    pad.SetLocalSolderMaskMargin(fmm(mask_margin))
    pad.SetLocalClearance(fmm(FID_CLEARANCE_MM))
    pad.SetNetCode(0)
    fp.Add(pad)

    crtyd = pcbnew.PCB_SHAPE(fp, pcbnew.SHAPE_T_RECT)
    crtyd_half = FID_RADIUS_MM + 0.1
    crtyd.SetStart(pcbnew.VECTOR2I(fmm(x - crtyd_half), fmm(y - crtyd_half)))
    crtyd.SetEnd(pcbnew.VECTOR2I(fmm(x + crtyd_half), fmm(y + crtyd_half)))
    crtyd.SetLayer(pcbnew.F_CrtYd)
    crtyd.SetWidth(fmm(0.05))
    fp.Add(crtyd)

    fp.Flip(pos, pcbnew.FLIP_DIRECTION_LEFT_RIGHT)

    set_uuid(fp, f"sh03:fiducial:{ref}:footprint")
    set_uuid(pad, f"sh03:fiducial:{ref}:pad")
    set_uuid(crtyd, f"sh03:fiducial:{ref}:courtyard")
    for field in fp.GetFields():
        set_uuid(field, f"sh03:fiducial:{ref}:field:{field.GetName()}")

    board.Add(fp)
    return fp, pad


def refill_bcu_gnd_zone(board):
    zones = pcbnew.ZONES()
    for zone in board.Zones():
        if zone.GetLayer() == pcbnew.B_Cu:
            zones.append(zone)
    if len(zones) == 0:
        raise SystemExit("[fiducials] no B.Cu zone found -- run add_ground_pour.py first")
    filler = pcbnew.ZONE_FILLER(board)
    ok = filler.Fill(zones)
    print(f"[fiducials] B.Cu zone refill: ZONE_FILLER.Fill() returned {ok}")
    return zones


def measure_pour_clearance(board, x, y):
    """Empirical clearance (mm) from the fiducial pad's copper edge to the
    nearest solid GND-pour copper, via a radial ZONE.HitTestFilledArea()
    sweep against the just-refilled B.Cu zone(s). Returns
    POUR_RADIUS_MAX_MM - FID_RADIUS_MM if nothing is hit within the sweep
    (i.e. "at least this clear")."""
    zones = [z for z in board.Zones() if z.GetLayer() == pcbnew.B_Cu]
    n_angles = round(360 / POUR_ANGLE_STEP_DEG)
    n_radii = round((POUR_RADIUS_MAX_MM - FID_RADIUS_MM) / POUR_RADIUS_STEP_MM)
    best_r = POUR_RADIUS_MAX_MM
    for ai in range(n_angles):
        theta = math.radians(ai * POUR_ANGLE_STEP_DEG)
        for ri in range(n_radii + 1):
            r = FID_RADIUS_MM + ri * POUR_RADIUS_STEP_MM
            if r >= best_r:
                break
            px = x + r * math.cos(theta)
            py = y + r * math.sin(theta)
            p = pcbnew.VECTOR2I(fmm(px), fmm(py))
            hit = any(z.HitTestFilledArea(pcbnew.B_Cu, p, 0) for z in zones)
            if hit:
                best_r = min(best_r, r)
                break
    return best_r - FID_RADIUS_MM


def main():
    board = pcbnew.LoadBoard(PCB_PATH)

    _kept_alive = []
    removed = remove_existing_fiducials(board, _kept_alive)
    print(f"[fiducials] removed {removed} existing fiducial footprint(s)")

    pad_rects = collect_bcu_pad_rects(board)
    track_segs = collect_bcu_track_segs(board)
    via_list = collect_vias(board)
    print(f"[fiducials] obstacle inventory: {len(pad_rects)} B.Cu pad(s), "
          f"{len(track_segs)} B.Cu track segment(s), {len(via_list)} via(s)")

    placements = {}
    for ref in FID_REFS:
        clearance, x, y = find_best_candidate(SEARCH_BOXES[ref], pad_rects, track_segs, via_list)
        placements[ref] = (x, y, clearance)
        print(f"[fiducials] {ref}: candidate ({x}, {y}) mm, fixed-obstacle clearance {clearance:.3f} mm")

    for ref in FID_REFS:
        x, y, _ = placements[ref]
        make_fiducial(board, ref, x, y)

    zones = refill_bcu_gnd_zone(board)

    print("[fiducials] --- final report ---")
    for ref in FID_REFS:
        x, y, fixed_clearance = placements[ref]
        pour_clearance = measure_pour_clearance(board, x, y)
        overall = min(fixed_clearance, pour_clearance)
        status = "OK" if overall >= FID_CLEARANCE_MM else "BELOW 2.0mm REQUIREMENT"
        print(f"[fiducials] {ref} at ({x}, {y}) mm: "
              f"fixed-obstacle clearance {fixed_clearance:.3f} mm, "
              f"GND-pour clearance {pour_clearance:.3f} mm, "
              f"overall {overall:.3f} mm -- {status}")

    board.Save(PCB_PATH)
    print(f"[fiducials] saved {PCB_PATH}")


if __name__ == "__main__":
    main()
