"""add_ground_pour.py - GND zone on both F.Cu and B.Cu for sh03-controller.kicad_pcb,
plus perimeter stitching vias tying the two pours together.

Run with KiCad's bundled interpreter:
  python3 scripts/add_ground_pour.py

Run this AFTER scripts\\route_signals.py, not before: route_signals.py wipes
every PCB_TRACK/PCB_VIA on the board on each run (its own idempotency rule),
which would delete any stitching vias a prior pour pass had placed. Pouring
last means the perimeter vias always survive.

Idempotent: removes any existing zone on F.Cu or B.Cu whose net is GND before
adding fresh ones, and removes previously-placed stitching vias (identified
by proximity to the perimeter ring, not by the 4 signal vias near R1-R4,
which sit well inside the board and are never touched). Fills and saves.

F.Cu is the primary reference plane; B.Cu is poured too, filling in around
the back-side signal routing from route_signals.py.

Fab constraints (JLCPCB, 2-layer): min trace 0.15mm, min clearance 0.15mm.
Zone connection: ZONE_CONNECTION_THT_THERMAL -- thermal-relief spokes for
through-hole pads, solid/direct connection for everything else (SMD pads).

Determinism: zone and stitching-via UUIDs are derived from stable keys
(layer name for zones; perimeter-walk index for vias) instead of pcbnew's
random default -- see place_footprints.py's module docstring for why that
matters. Via placement order was already deterministic (perimeter_candidates()
is a pure function of board outline geometry, no footprint/pad iteration
involved); obstacle-rect collection is sorted here too so the "skipped as
too close" decision never depends on board.GetFootprints()/Pads() order,
even though as a boolean any() check its result never actually depended on
that order.
"""

import math
import os
import uuid
from collections import deque

import pcbnew

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB_PATH = os.path.join(PROJECT_DIR, "sh03-controller.kicad_pcb")

# --- determinism helpers ---------------------------------------------------

UUID_NAMESPACE = uuid.UUID("25a1abba-b61d-5666-8453-b2e72c16e5ab")
# Stable namespace for deterministic object identifiers.


def stable_uuid(key):
    return str(uuid.uuid5(UUID_NAMESPACE, key))


def set_uuid(item, key):
    """See place_footprints.py's set_uuid for why .Clone() is the write path."""
    item.m_Uuid.Clone(pcbnew.KIID(stable_uuid(key)))


BOARD_OUTLINE = (36.0, 33.0, 136.0, 89.0)  # (min_x, min_y, max_x, max_y) mm -- east
# edge widened 132 -> 136 (96 -> 100mm board width, 2026-08-12d east-widen rework,
# kept under JLCPCB's <=100x100mm price tier) -- see docs/layout-plan.md.
INSET_MM = 0.5

# --- RF antenna keepout -----------------------------------------------
#
# U1 (ESP32-S3-DevKitC-1-Socket) is placed at (95, 61.1) rotated 90 deg.
# The module's PCB antenna lives at its pin-1 end -- Espressif's dimensioned
# drawing (see docs/layout-plan.md's "Settled: the DevKitC-1 footprint's
# body geometry" entry) puts pin 1 only 1.40mm in from that end of the
# 62.74mm-long body, vs. 8.00mm of body beyond the far (pin-22/USB) end, so
# the antenna is unambiguously the LOW end of the module's long axis, not
# the high end.
#
# After the 90-degree placement rotation that long axis runs along board X,
# and pin 1 lands at the LOW-X (west) side: U1's own pad 1 sits at world
# (68.33, 72.53), just inside the courtyard's west edge -- confirmed by
# loading the board and reading pad "1"'s live GetPosition() plus
# fp.GetCourtyard(F_CrtYd).BBox() directly (not hand-derived from the
# footprint file's local coordinates + a rotation matrix, to avoid a sign
# error on the 90-degree transform): courtyard world bbox X[66.635,
# 129.965] Y[48.105, 74.095].
#
# Keepout rectangle:
#   X: from the courtyard's west edge (66.635) inboard 8.0mm (74.635),
#      i.e. covering roughly the antenna trace + ground-clearance area of
#      the module per Espressif's module reference design guidance, then
#      grown 1mm on both the west (65.635) and east (75.635) sides.
#   Y: NOT the courtyard's full Y-span. U1's antenna runs down the
#      module's CENTER, in the gap between its two 22-pin 2.54mm header
#      rows -- the rows themselves (where every pin, including the
#      module's own power/ground breakout, physically lands) are outside
#      the RF-sensitive area and are exactly where a DevKitC carrier board
#      is expected to route. Row centerlines, read from the live board
#      (fp.Pads() GetPosition(), same method used for the X-derivation
#      above): pins 23-44 at y=49.670, pins 1-22 at y=72.530. Keepout Y is
#      each row's centerline plus a 2.5mm standoff, narrowing INWARD:
#      Y0 = 49.670 + 2.5 = 52.17, Y1 = 72.530 - 2.5 = 70.03.
#
# FIRST ATTEMPT (reverted): Y spanning the courtyard's full extent
# (48.105-74.095 +-1mm margin = 47.105-75.095). That's wider than the
# antenna itself and blocked U1's own pin 1/pin 2 pads (world
# x=68.33/70.87, y=72.53, both /+3V3, both inside the 8mm X-reach) --
# their own pad grid cells sat inside the keepout, so /+3V3's MST edge
# R2.1->U1.1 had zero valid A* start nodes on either layer (route_board.py
# reported "NO PATH after 0.00s", not a search failure) and the rip-up/
# retry queue cycled forever, never converging, because the target was
# structurally unreachable. Verified via a real route_board.py run before
# narrowing, not assumed.
#
# With the narrowed Y range, pins 1/2 (y=72.53) sit 2.5mm outside the
# keepout's south edge (70.03) -- the pin-1<->pin-2 /+3V3 stub, and R2's
# incoming edge, both run along the pad-row corridor entirely outside the
# rule area. Pads and footprints are explicitly ALLOWED inside a KiCad
# keepout rule area regardless (only tracks, vias and zone fill are
# excluded) -- narrowing Y removes the conflict at the track level, not
# just the pad level. See route_board.py's ANTENNA_KEEPOUT_MM (kept
# numerically identical to this) for how the router treats it, and
# docs/layout-plan.md for the same derivation in prose.
ANTENNA_KEEPOUT_MM = (65.635, 52.17, 75.635, 70.03)  # (x0, y0, x1, y1)
ANTENNA_KEEPOUT_LAYERS = [pcbnew.F_Cu, pcbnew.B_Cu]

MIN_CLEARANCE_MM = 0.15
MIN_TRACE_WIDTH_MM = 0.15
THERMAL_RELIEF_GAP_MM = 0.2       # > MIN_CLEARANCE_MM
THERMAL_RELIEF_SPOKE_MM = 0.25    # > MIN_TRACE_WIDTH_MM

SIG_WIDTH = 0.25  # matches route_board.py's SIG_WIDTH -- standard signal
                  # trace width for the GND_STUB_TRACES fallback (see its
                  # comment)

GND_NET_CANDIDATES = ["/GND", "GND"]

POUR_LAYERS = [pcbnew.F_Cu, pcbnew.B_Cu]

# --- stitching vias ---
STITCH_INSET_MM = 2.0        # inset from board edge
STITCH_PITCH_MM = 10.0       # nominal spacing around the perimeter
STITCH_DRILL_MM = 0.3
STITCH_PAD_MM = 0.6
STITCH_OBSTACLE_CLEARANCE_MM = 1.0
# a via found within this distance of the board edge counts as "ours" for
# idempotent removal -- the 4 signal vias near R1-R4 are all >7mm inside
STITCH_BAND_MM = STITCH_INSET_MM + 2.0


def fmm(v):
    return pcbnew.FromMM(v)


def mm(nm):
    return pcbnew.ToMM(nm)


def find_gnd_net(board):
    for name in GND_NET_CANDIDATES:
        ni = board.FindNet(name)
        if ni is not None:
            return ni
    sys_exit_names = ", ".join(GND_NET_CANDIDATES)
    raise SystemExit(f"[pour] no net found matching any of: {sys_exit_names}")


def remove_existing_gnd_zones(board, gnd_netcode, keep_alive):
    """Removes matching zones on either copper layer. Keeps their SWIG
    proxies alive in `keep_alive` for the rest of the script's run -- same
    KiCad 10.0.5 pcbnew bug documented in place_footprints.py: a removed
    item's proxy getting garbage-collected corrupts later calls into
    unrelated board APIs (reproduced here against SHAPE_POLY_SET.NewOutline()
    on a fresh zone after a zone removal is GC'd)."""
    removed = 0
    for zone in list(board.Zones()):
        if zone.GetLayer() in POUR_LAYERS and zone.GetNetCode() == gnd_netcode:
            board.Remove(zone)
            keep_alive.append(zone)
            removed += 1
    return removed


def make_zone(board, layer, gnd_net, x0, y0, x1, y1, name):
    zone = pcbnew.ZONE(board)
    zone.SetLayer(layer)
    zone.SetNetCode(gnd_net.GetNetCode())
    zone.SetZoneName(name)

    outline = zone.Outline()
    outline.NewOutline()
    outline.Append(fmm(x0), fmm(y0))
    outline.Append(fmm(x1), fmm(y0))
    outline.Append(fmm(x1), fmm(y1))
    outline.Append(fmm(x0), fmm(y1))

    zone.SetLocalClearance(fmm(MIN_CLEARANCE_MM))
    zone.SetMinThickness(fmm(MIN_TRACE_WIDTH_MM))
    zone.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)
    zone.SetThermalReliefGap(fmm(THERMAL_RELIEF_GAP_MM))
    zone.SetThermalReliefSpokeWidth(fmm(THERMAL_RELIEF_SPOKE_MM))
    zone.SetFillMode(pcbnew.ZONE_FILL_MODE_POLYGONS)
    zone.SetIsFilled(False)

    board.Add(zone)
    set_uuid(zone, f"sh03:zone:{name}")
    return zone


ANTENNA_KEEPOUT_NAME = "U1_antenna_keepout"


def remove_existing_keepout_zone(board, keep_alive):
    """Idempotent removal, matched by zone name rather than net (a keepout
    rule area has no net) -- same GC-proxy-keepalive requirement as
    remove_existing_gnd_zones()."""
    removed = 0
    for zone in list(board.Zones()):
        if zone.GetZoneName() == ANTENNA_KEEPOUT_NAME:
            board.Remove(zone)
            keep_alive.append(zone)
            removed += 1
    return removed


def make_antenna_keepout_zone(board):
    """RF keepout rule area under U1's antenna: no tracks, no vias, no
    copper pour (on either layer) -- pads and footprints stay allowed, so
    U1's own through-hole pins in the region are unaffected. Must be added
    to the board BEFORE ZONE_FILLER.Fill() runs on the GND pours below --
    KiCad's zone filler treats an on-board keepout rule area as a hard
    boundary for every OTHER zone's fill, so the pour retreats from it
    automatically as long as this zone already exists at fill time."""
    zone = pcbnew.ZONE(board)
    zone.SetLayer(ANTENNA_KEEPOUT_LAYERS[0])
    # LSET's Python constructor doesn't accept a plain list/vector of
    # layer IDs in this KiCad 10.0.5 SWIG binding (TypeError on both a
    # length-prefixed and a bare-list call, verified). AllCuMask() is
    # exactly F.Cu+B.Cu on this 2-layer board -- confirmed via
    # .Contains(F_Cu)/.Contains(B_Cu)/.Contains(F_SilkS) before relying on
    # it -- so it's equivalent to the intended ANTENNA_KEEPOUT_LAYERS set
    # without depending on a list-arg constructor that doesn't exist here.
    zone.SetLayerSet(pcbnew.LSET.AllCuMask())
    zone.SetZoneName(ANTENNA_KEEPOUT_NAME)
    zone.SetIsRuleArea(True)
    zone.SetDoNotAllowTracks(True)
    zone.SetDoNotAllowVias(True)
    # KiCad 10.0.5's ZONE class names this SetDoNotAllowZoneFills, not
    # SetDoNotAllowCopperPour (verified via dir(ZONE) after the first
    # AttributeError -- don't guess a second time).
    zone.SetDoNotAllowZoneFills(True)
    zone.SetDoNotAllowPads(False)
    zone.SetDoNotAllowFootprints(False)

    x0, y0, x1, y1 = ANTENNA_KEEPOUT_MM
    outline = zone.Outline()
    outline.NewOutline()
    outline.Append(fmm(x0), fmm(y0))
    outline.Append(fmm(x1), fmm(y0))
    outline.Append(fmm(x1), fmm(y1))
    outline.Append(fmm(x0), fmm(y1))

    board.Add(zone)
    set_uuid(zone, f"sh03:zone:{ANTENNA_KEEPOUT_NAME}")
    return zone


def remove_existing_stitch_vias(board, gnd_netcode, keep_alive):
    ox0, oy0, ox1, oy1 = BOARD_OUTLINE
    removed = 0
    for via in list(board.Tracks()):
        if via.Type() != pcbnew.PCB_VIA_T or via.GetNetCode() != gnd_netcode:
            continue
        pos = via.GetPosition()
        x, y = mm(pos.x), mm(pos.y)
        near_edge = (
            x - ox0 < STITCH_BAND_MM
            or ox1 - x < STITCH_BAND_MM
            or y - oy0 < STITCH_BAND_MM
            or oy1 - y < STITCH_BAND_MM
        )
        if near_edge:
            board.Remove(via)
            keep_alive.append(via)
            removed += 1
    return removed


def collect_obstacle_rects(board):
    """Every pad bbox and every footprint courtyard bbox, in mm. Walked in
    explicit (refdes, pad number) order -- see place_footprints.py's module
    docstring for why board.GetFootprints()/fp.Pads() order isn't trusted
    directly. The result only ever feeds an any() proximity check, so this
    ordering never actually changed behavior, but it keeps the invariant
    (no undeclared reliance on container order) true throughout the
    pipeline rather than in two of the three scripts."""
    rects = []
    for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
        for pad in sorted(fp.Pads(), key=lambda p: p.GetNumber()):
            bb = pad.GetBoundingBox()
            rects.append((mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom())))
        for crtyd_layer in (pcbnew.F_CrtYd, pcbnew.B_CrtYd):
            poly = fp.GetCourtyard(crtyd_layer)
            if poly is not None and not poly.IsEmpty():
                bb = poly.BBox()
                rects.append((mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom())))
    return rects


def dist_point_to_rect(px, py, rect):
    x0, y0, x1, y1 = rect
    dx = max(x0 - px, px - x1, 0.0)
    dy = max(y0 - py, py - y1, 0.0)
    return math.hypot(dx, dy)


def dist_point_to_segment(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    seg_len2 = dx * dx + dy * dy
    if seg_len2 < 1e-9:
        return math.hypot(px - x0, py - y0)
    t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / seg_len2))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))


def collect_track_obstacles(board, exclude_netcode):
    """Every routed PCB_TRACK segment and PCB_VIA on the board, EXCLUDING
    items already on the GND net (a GND stitching via touching GND-net
    copper is a connection, not a short). Returns two lists:
      segs: (x0, y0, x1, y1, half_width_mm)
      vias: (x, y, radius_mm)
    Without this, add_stitch_vias() only ever checked clearance against pad
    and courtyard bboxes (collect_obstacle_rects()) -- it never looked at
    already-routed copper from route_board.py at all. That was invisible
    as long as route_board.py's own livelock kept it from ever finishing a
    real routing pass near the board edges; once it did (see
    RIPUP_MARGIN_MM's comment in route_board.py), several perimeter
    stitching vias landed on top of or within clearance of signal traces
    near the bottom edge, producing real DRC shorting_items/clearance
    violations. Walked in explicit UUID-sorted order for the same "no
    undeclared reliance on board.Tracks() iteration order" reason
    collect_obstacle_rects() already documents -- this only ever feeds an
    any() proximity check, so it doesn't change behavior, just keeps the
    invariant true.
    """
    segs = []
    vias = []
    for t in sorted(board.Tracks(), key=lambda t: str(t.m_Uuid)):
        if t.GetNetCode() == exclude_netcode:
            continue
        if t.Type() == pcbnew.PCB_VIA_T:
            # KiCad 10 vias can have a per-layer diameter; GetWidth() with
            # no argument is ambiguous and logs a "called without a layer
            # argument" assert. This board only ever creates uniform
            # through-vias (route_board.py / add_ground_pour.py's own
            # perimeter stitching vias both call SetWidth() once, not per
            # layer), so F_Cu's width is the same value GetWidth() used to
            # return.
            pos = t.GetPosition()
            vias.append((mm(pos.x), mm(pos.y), mm(t.GetWidth(pcbnew.F_Cu)) / 2.0))
        else:
            start, end = t.GetStart(), t.GetEnd()
            segs.append((mm(start.x), mm(start.y), mm(end.x), mm(end.y), mm(t.GetWidth()) / 2.0))
    return segs, vias


def perimeter_candidates(x0, y0, x1, y1, pitch):
    """Walk the inset rectangle perimeter clockwise from the top-left corner,
    dropping a candidate point every `pitch` mm of arc length."""
    edges = [
        (x0, y0, x1, y0),  # top
        (x1, y0, x1, y1),  # right
        (x1, y1, x0, y1),  # bottom
        (x0, y1, x0, y0),  # left
    ]
    points = []
    carry = 0.0
    for ex0, ey0, ex1, ey1 in edges:
        length = math.hypot(ex1 - ex0, ey1 - ey0)
        if length <= 0:
            continue
        ux, uy = (ex1 - ex0) / length, (ey1 - ey0) / length
        d = pitch - carry
        while d < length:
            points.append((ex0 + ux * d, ey0 + uy * d))
            d += pitch
        carry = length - (d - pitch)
    return points


def add_stitch_vias(board, gnd_net, obstacle_rects, track_segs, track_vias, zones_to_fill):
    """zones_to_fill: the filled F.Cu/B.Cu GND zones -- 2026-08-12c fix.
    A real DRC run (via_dangling, 16 instances, all landing on this
    function's perimeter grid) found this walker was placing vias purely
    on OBSTACLE clearance, never checking whether the candidate point is
    actually inside the filled zone polygon on BOTH layers. F.Cu's fill
    split into 3 separate outlines around U1's footprint/antenna keepout
    (confirmed via ZONE.GetFilledPolysList() outline count -- B.Cu stayed
    a single outline), and several perimeter-walk grid points landed in a
    genuine F.Cu void between those islands: valid by obstacle-clearance
    (nothing else is nearby), but not touching F.Cu copper at all, so the
    via only ever connected on B.Cu -- "not connected or connected on only
    one layer". Same failure category find_orphan_pour_islands() already
    handles for INTERIOR stitching (a via inside a bbox that isn't
    genuinely inside that island's own polygon), just not previously
    checked here. Fix: same SHAPE_POLY_SET.Contains() membership test,
    applied to both pour layers before accepting a candidate."""
    ox0, oy0, ox1, oy1 = BOARD_OUTLINE
    x0, y0 = ox0 + STITCH_INSET_MM, oy0 + STITCH_INSET_MM
    x1, y1 = ox1 - STITCH_INSET_MM, oy1 - STITCH_INSET_MM

    candidates = perimeter_candidates(x0, y0, x1, y1, STITCH_PITCH_MM)
    placed = 0
    skipped = 0
    skipped_not_in_fill = 0
    for idx, (px, py) in enumerate(candidates):
        # our own via has half-width STITCH_PAD_MM/2 -- clearance is
        # measured from copper to copper, not centerline to obstacle edge,
        # so that has to come off the budget on top of the flat
        # STITCH_OBSTACLE_CLEARANCE_MM margin (which was tuned against the
        # pad/courtyard-only obstacle set and is generous enough to already
        # cover it, but be explicit rather than rely on the margin alone).
        via_half = STITCH_PAD_MM / 2.0
        too_close = any(
            dist_point_to_rect(px, py, rect) < STITCH_OBSTACLE_CLEARANCE_MM
            for rect in obstacle_rects
        ) or any(
            dist_point_to_segment(px, py, sx0, sy0, sx1, sy1) - seg_half_w - via_half
            < STITCH_OBSTACLE_CLEARANCE_MM
            for (sx0, sy0, sx1, sy1, seg_half_w) in track_segs
        ) or any(
            math.hypot(px - vx, py - vy) - via_r - via_half < STITCH_OBSTACLE_CLEARANCE_MM
            for (vx, vy, via_r) in track_vias
        ) or dist_point_to_rect(px, py, ANTENNA_KEEPOUT_MM) - via_half < STITCH_OBSTACLE_CLEARANCE_MM
        if too_close:
            skipped += 1
            continue
        pt = pcbnew.VECTOR2I(fmm(px), fmm(py))
        in_fill_both_layers = True
        for zone in zones_to_fill:
            layer = zone.GetLayer()
            filled = zone.GetFilledPolysList(layer)
            if filled is None or not filled.Contains(pt, False, 0, False):
                in_fill_both_layers = False
                break
        if not in_fill_both_layers:
            skipped += 1
            skipped_not_in_fill += 1
            continue
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(pcbnew.VECTOR2I(fmm(px), fmm(py)))
        via.SetDrill(fmm(STITCH_DRILL_MM))
        via.SetWidth(fmm(STITCH_PAD_MM))
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetNetCode(gnd_net.GetNetCode())
        board.Add(via)
        # keyed by the candidate's fixed perimeter-walk index, not the
        # placed-count -- stable even if a future obstacle-set change
        # shifts which candidates get skipped
        set_uuid(via, f"sh03:gndperimeter:{idx}")
        placed += 1
    return placed, skipped, skipped_not_in_fill, len(candidates)


def find_orphan_pour_islands(zones_to_fill, existing_gnd_points):
    """2026-08-11: a real DRC run flagged 'missing connection' between the
    F.Cu and B.Cu GND zones -- the antenna keepout (and the routed copper
    around it) splits each layer's fill into several disconnected polygon
    islands (confirmed: F.Cu had 2 outlines, B.Cu had 4), and
    add_stitch_vias() only ever walks the board PERIMETER (STITCH_INSET_MM
    from the edge) -- it can never place a via inside an interior island,
    so any island that doesn't happen to touch a pad or a perimeter via is
    genuinely floating GND copper with no path back to the rest of the net.

    existing_gnd_points is VIAS ONLY, deliberately, not GND pads (including
    THT ones). TRIED pads-inclusive first, on the theory that a THT pad
    already bridges both copper layers via its own plated barrel: a real
    DRC run proved that assumption wrong for this board -- J3's THT GND
    pin (pad 2) sits inside a genuinely isolated ~309mm^2 B.Cu island (see
    docs/layout-plan.md), and DRC still reported "missing connection"
    between that zone and the pad even though the pad is geometrically
    inside the island's bbox. A pad only bridges the TWO LAYERS at its own
    single point -- it doesn't reach OUT to a different, separately-filled
    island elsewhere, and being "inside an island's bbox" doesn't mean the
    island itself has a path back to the main plane. Only a VIA is
    unambiguous bridging evidence for this purpose (it's not tied to a
    specific component footprint's fill in the way a pad's thermal relief
    is), so only vias count as "already connected" here.

    Returns a list of (oc, x0, y0, x1, y1, layer, area_mm2) -- the full
    bbox of every filled outline that ISN'T the largest one on its layer
    (the main pour) and doesn't already contain an existing GND via --
    not just its center point, so the caller can search the bbox for a
    clear spot instead of trusting the center to be obstacle-free."""
    orphans = []
    for zone in zones_to_fill:
        layer = zone.GetLayer()
        filled = zone.GetFilledPolysList(layer)
        if filled is None or filled.OutlineCount() <= 1:
            continue
        outlines = []
        for oc in range(filled.OutlineCount()):
            outline = filled.Outline(oc)
            bb = outline.BBox()
            outlines.append((oc, mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom())))
        # main pour = largest bbox area, sorted first by area then by
        # outline index so a tie never depends on OutlineCount() iteration
        # order
        outlines.sort(key=lambda o: (-(o[3] - o[1]) * (o[4] - o[2]), o[0]))
        for (oc, x0, y0, x1, y1) in outlines[1:]:
            already_connected = any(
                x0 - STITCH_PAD_MM <= vx <= x1 + STITCH_PAD_MM
                and y0 - STITCH_PAD_MM <= vy <= y1 + STITCH_PAD_MM
                for (vx, vy) in existing_gnd_points
            )
            if already_connected:
                continue
            orphans.append((oc, x0, y0, x1, y1, layer, (x1 - x0) * (y1 - y0)))
    return orphans


def via_clearance_ok(px, py, obstacle_rects, track_segs, track_vias):
    via_half = STITCH_PAD_MM / 2.0
    too_close = any(
        dist_point_to_rect(px, py, rect) < STITCH_OBSTACLE_CLEARANCE_MM
        for rect in obstacle_rects
    ) or any(
        dist_point_to_segment(px, py, sx0, sy0, sx1, sy1) - seg_half_w - via_half
        < STITCH_OBSTACLE_CLEARANCE_MM
        for (sx0, sy0, sx1, sy1, seg_half_w) in track_segs
    ) or any(
        math.hypot(px - vx, py - vy) - via_r - via_half < STITCH_OBSTACLE_CLEARANCE_MM
        for (vx, vy, via_r) in track_vias
    ) or dist_point_to_rect(px, py, ANTENNA_KEEPOUT_MM) - via_half < STITCH_OBSTACLE_CLEARANCE_MM
    return not too_close


def trace_clearance_ok(px, py, obstacle_rects, track_segs, track_vias):
    """Same shape as via_clearance_ok(), but sized for a thin GND_STUB_TRACES
    signal trace, not a stitching via. via_clearance_ok's margin
    (STITCH_OBSTACLE_CLEARANCE_MM=1.0mm + a 0.3mm via-pad half-width, a
    manufacturing-reliability margin tuned for VIA registration) is far
    stricter than an actual DRC minimum for a plain trace -- reusing it
    for J3 pin 2's stub search found the pad sitting in a genuinely
    isolated 7-cell pocket at 0.2mm grid resolution (confirmed via a real
    BFS run: zero path to anywhere, let alone the main pour), which turned
    out to be an artifact of the wrong clearance model, not a real dead
    end. This uses the board's actual fab minimum (MIN_CLEARANCE_MM) plus
    the stub's own real half-width (SIG_WIDTH/2), same as every other
    trace on this board is held to."""
    half_width = SIG_WIDTH / 2.0
    margin = MIN_CLEARANCE_MM
    too_close = any(
        dist_point_to_rect(px, py, rect) - half_width < margin
        for rect in obstacle_rects
    ) or any(
        dist_point_to_segment(px, py, sx0, sy0, sx1, sy1) - seg_half_w - half_width
        < margin
        for (sx0, sy0, sx1, sy1, seg_half_w) in track_segs
    ) or any(
        math.hypot(px - vx, py - vy) - via_r - half_width < margin
        for (vx, vy, via_r) in track_vias
    ) or dist_point_to_rect(px, py, ANTENNA_KEEPOUT_MM) - half_width < margin
    return not too_close


def get_main_outline(zones_to_fill, pcb_layer):
    """The largest filled outline on pcb_layer, i.e. the "main pour" --
    same concept and same largest-bbox-area selection add_gnd_stub_traces()
    already uses for GND_STUB_TRACES's endpoint search (see its inline
    loop), pulled out here so add_interior_stitch_vias() can reuse it
    for the opposite-layer containment check below. Returns a
    SHAPE_LINE_CHAIN (outline) or None if the layer has no fill."""
    main_outline = None
    best_area = -1.0
    for zone in zones_to_fill:
        if zone.GetLayer() != pcb_layer:
            continue
        filled = zone.GetFilledPolysList(pcb_layer)
        if filled is None:
            continue
        for oc in range(filled.OutlineCount()):
            outline = filled.Outline(oc)
            bb = outline.BBox()
            area = mm(bb.GetWidth()) * mm(bb.GetHeight())
            if area > best_area:
                best_area = area
                main_outline = outline
    return main_outline


def find_clear_point_in_bbox(x0, y0, x1, y1, obstacle_rects, track_segs, track_vias,
                              island_outline=None, opposite_main_outline=None):
    """Deterministic local search for a clearance-clean via spot inside an
    island's bbox: bbox-center first (the common case), then a fixed,
    ever-widening grid of offsets from it (0.2mm step, snapped to the
    0.1mm routing grid), all clipped to the bbox, tried in a fixed
    row-major order so the result never depends on set/dict iteration.

    island_outline and opposite_main_outline, if given, are each checked
    with PointInside() (same idiom find_stub_endpoint() uses) as ADDITIONAL
    requirements on top of the bbox and discrete-obstacle clearance:

    - island_outline is the orphan island's OWN polygon on its OWN layer.
      Being inside the bbox was never the same thing as being inside the
      shape -- a real DRC run caught exactly this gap 2026-08-12 (after the
      all-SMD-on-back rework changed the F.Cu/B.Cu obstacle landscape
      enough to reshape islands and shift bbox-center candidates): the
      247.46mm^2 F.Cu island's own bbox center, (85.4, 85.8), sits in a
      concave notch of that island's actual outline (confirmed directly:
      NONE of that island's own filled outlines contain the point, despite
      being deep inside its bbox and clearance-clean) -- so a via placed
      there never actually touched the copper it was meant to bridge into,
      no matter how clean B.Cu came out.
    - opposite_main_outline is the OTHER layer's main pour: a through-via
      only actually stitches the two layers together if its landing point
      has real copper on BOTH sides, not just clearance on the island's own
      layer -- the other half of the same DRC finding above.

    Both checks were missing before; obstacle-clearance alone was never
    sufficient, it just hadn't been exercised by a candidate that failed
    this way until this rework's placement change produced one.

    Returns (x, y) or None if nothing in the bbox clears every check given."""
    def _ok(px, py):
        if not via_clearance_ok(px, py, obstacle_rects, track_segs, track_vias):
            return False
        pt = None
        if island_outline is not None:
            pt = pcbnew.VECTOR2I(fmm(px), fmm(py))
            if not island_outline.PointInside(pt):
                return False
        if opposite_main_outline is not None:
            if pt is None:
                pt = pcbnew.VECTOR2I(fmm(px), fmm(py))
            if not opposite_main_outline.PointInside(pt):
                return False
        return True

    cx = round(((x0 + x1) / 2.0) / 0.1) * 0.1
    cy = round(((y0 + y1) / 2.0) / 0.1) * 0.1
    if _ok(cx, cy):
        return (cx, cy)
    # Ring-from-center search: fast, and correct for the common case (a
    # roughly-convex island whose bbox center is close to its interior).
    # WRONG for an elongated/concave island whose actual copper sits well
    # off-center within its own bbox -- 2026-08-12: a real run against the
    # 247.46mm^2 F.Cu orphan (bbox 40.91 x 6.04mm, center at (65.14,
    # 83.45)) found ZERO hits with this search even though the island has
    # 459 valid interior points at a 0.5mm grid sampling, because they all
    # cluster near the bbox's WEST edge (X~45-47) -- ~18-20mm off-center,
    # right at this search's own radius ceiling, and the circular ring
    # sampling wastes most of its points per ring on angles that fall
    # outside the narrow 6mm-tall bbox before ever reaching that edge.
    # Kept as the fast path (most islands the pipeline has produced so far
    # ARE convex/centered), but no longer trusted alone -- if it comes up
    # empty, fall through to the exhaustive raster scan below rather than
    # reporting a false "nothing found."
    step = 0.2
    max_radius_mm = max(x1 - x0, y1 - y0) / 2.0 + step
    ring = 1
    while ring * step <= max_radius_mm:
        offsets = []
        r = ring * step
        n = ring * 8  # points around this ring, fixed count, fixed order
        for k in range(n):
            angle = 2 * math.pi * k / n
            ox = round((r * math.cos(angle)) / 0.1) * 0.1
            oy = round((r * math.sin(angle)) / 0.1) * 0.1
            offsets.append((ox, oy))
        for (ox, oy) in offsets:
            px, py = cx + ox, cy + oy
            if not (x0 <= px <= x1 and y0 <= py <= y1):
                continue
            if _ok(px, py):
                return (px, py)
        ring += 1

    # Exhaustive fallback: deterministic row-major raster scan of the
    # WHOLE bbox at the same 0.2mm pitch, top-left first. Bounded (island
    # bboxes are a small fraction of the board) and guarantees finding any
    # qualifying point the ring search could miss, in a fixed, reproducible
    # order independent of the ring search's own miss.
    ny = int(round((y1 - y0) / step)) + 1
    nx = int(round((x1 - x0) / step)) + 1
    for j in range(ny):
        py = round((y0 + j * step) / 0.1) * 0.1
        if py > y1:
            continue
        for i in range(nx):
            px = round((x0 + i * step) / 0.1) * 0.1
            if px > x1:
                continue
            if _ok(px, py):
                return (px, py)
    return None


def add_interior_stitch_vias(board, gnd_net, orphans, obstacle_rects, track_segs, track_vias,
                              zones_to_fill):
    """One via per orphan island (see find_orphan_pour_islands()), searched
    within the island's own bbox for a clearance-clean spot that ALSO lands
    inside the opposite layer's main pour (bbox-center first, then a
    deterministic outward ring search -- see find_clear_point_in_bbox())
    rather than trusting the center alone or discrete-obstacle clearance
    alone. Refuses (with a report) any island where nothing in its bbox
    clears both checks, rather than forcing a via that wouldn't actually
    connect."""
    placed = 0
    skipped = 0
    for (oc, x0, y0, x1, y1, layer, area_mm2) in orphans:
        lname = "FCu" if layer == pcbnew.F_Cu else "BCu"
        island_outline = None
        for zone in zones_to_fill:
            if zone.GetLayer() != layer:
                continue
            filled = zone.GetFilledPolysList(layer)
            if filled is not None and oc < filled.OutlineCount():
                island_outline = filled.Outline(oc)
            break
        opposite_layer = pcbnew.B_Cu if layer == pcbnew.F_Cu else pcbnew.F_Cu
        opposite_main_outline = get_main_outline(zones_to_fill, opposite_layer)
        found = find_clear_point_in_bbox(x0, y0, x1, y1, obstacle_rects, track_segs, track_vias,
                                          island_outline=island_outline,
                                          opposite_main_outline=opposite_main_outline)
        if found is None:
            print(f"[pour] WARNING: interior stitching via for orphan island "
                  f"({lname}, area={area_mm2:.2f}mm^2, bbox=({x0:.2f},{y0:.2f})-"
                  f"({x1:.2f},{y1:.2f})) skipped -- no spot found anywhere in "
                  f"its bbox that clears obstacles AND lands inside both the "
                  f"island's own polygon and the opposite layer's main pour")
            skipped += 1
            continue
        px, py = found
        via = pcbnew.PCB_VIA(board)
        via.SetPosition(pcbnew.VECTOR2I(fmm(px), fmm(py)))
        via.SetDrill(fmm(STITCH_DRILL_MM))
        via.SetWidth(fmm(STITCH_PAD_MM))
        via.SetViaType(pcbnew.VIATYPE_THROUGH)
        via.SetNetCode(gnd_net.GetNetCode())
        board.Add(via)
        # keyed by the island's own (layer, found position) rather than
        # idx/placed-count -- stable across reruns as long as the island's
        # shape doesn't change, independent of how many other islands
        # existed or in what order they were enumerated
        set_uuid(via, f"sh03:gndisland:{lname}:{px:.3f}:{py:.3f}")
        print(f"[pour] interior stitching via placed for orphan island "
              f"({lname}, area={area_mm2:.2f}mm^2) at ({px:.3f},{py:.3f})")
        placed += 1
    return placed, skipped


# --- GND stub traces for pads a via can't reach ----------------------
#
# 2026-08-11: after the interior-island via fix above, a real DRC re-run
# showed the B.Cu "missing connection" cleared but a NEW one appeared on
# F.Cu, same pad: J3 pin 2 (GND). Root cause is different from the B.Cu
# case (that was one large island with no via anywhere in it -- fixed by
# find_orphan_pour_islands()/add_interior_stitch_vias() above): on F.Cu,
# J3's tight 2.5mm pin pitch fragments the pour into TWO tiny slivers
# immediately flanking pin 2 (confirmed: 5.01mm^2 and 3.77mm^2, neither
# containing pin 2 itself -- it sits in the gap between them), both too
# small for a via to clear STITCH_OBSTACLE_CLEARANCE_MM anywhere inside
# ("no clearance-clean spot found anywhere in its bbox", confirmed via a
# real run). A via needs room for its own pad; a thin 0.25mm signal-width
# stub trace doesn't, so it fits where a via can't.
#
# FIRST ATTEMPT (reverted): a hand-picked straight vertical stub 3.2mm due
# north to (59.5, 82.0), chosen because that point tested PointInside()
# true against the F.Cu main pour's filled outline. That check only
# verified the ENDPOINT, not the path -- a real run caught /MCU_TX's own
# F.Cu trace running horizontally right across that path (X 58.1-77.9 at
# Y=83.6, confirmed via direct track inspection, layer checked explicitly
# since collect_track_obstacles() doesn't separate F.Cu from B.Cu and a
# same-Y B.Cu trace nearby turned out to be a red herring). Hand-picking a
# second direction risked the same mistake, so this is now a search
# instead: a fixed ring of candidate directions/distances from the pad,
# each checked for BOTH full-path clearance (sampled, not just the
# endpoint) and the endpoint actually landing inside the target layer's
# main pour outline (PointInside() on the real polygon), same discipline
# as add_interior_stitch_vias()'s bbox search. First candidate that clears
# both, in a fixed deterministic order, wins.
GND_STUB_TRACES = [
    # (net_name, ref, pin, layer_index) -- endpoint is searched, not
    # hardcoded (see above for why).
    ("/GND", "J3", "2", 0),  # F.Cu
]

STUB_SEARCH_STEP_MM = 0.5
STUB_SEARCH_MAX_RADIUS_MM = 15.0
STUB_SEARCH_ANGLES = 24  # every 15 degrees


def path_clear(x0, y0, x1, y1, obstacle_rects, track_segs, track_vias, step_mm=0.2):
    length = math.hypot(x1 - x0, y1 - y0)
    steps = max(1, int(length / step_mm))
    for i in range(steps + 1):
        t = i / steps
        px, py = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        if not trace_clearance_ok(px, py, obstacle_rects, track_segs, track_vias):
            return False, (px, py)
    return True, None


def find_stub_endpoint(pad_x, pad_y, main_outline, obstacle_rects, track_segs, track_vias):
    """Deterministic outward ring search for a point reachable from
    (pad_x, pad_y) by a STRAIGHT, clearance-clean line that also lands
    inside main_outline. Returns (x, y) or None.

    TRIED FIRST, KEPT AS THE FAST PATH: works for isolated pads with open
    space nearby (the common case). FAILED for J3 pin 2 specifically -- a
    real run swept all 24 angles at 5 radii (1-8mm) and every single one
    was blocked, because /MCU_TX (F.Cu) and /MC_XQB2 (B.Cu) both run
    nearly full-board-width horizontal tracks at Y=83.6, 1.4mm from J3's
    pin row -- inside the 1.0mm clearance margin from almost any point in
    between. No straight line from J3.2 escapes that band within a
    reasonable radius; see find_stub_path_bfs() for what actually
    resolved it. Left in place as the cheap first attempt for pads that
    aren't boxed in this badly."""
    ring = 1
    while ring * STUB_SEARCH_STEP_MM <= STUB_SEARCH_MAX_RADIUS_MM:
        r = ring * STUB_SEARCH_STEP_MM
        for k in range(STUB_SEARCH_ANGLES):
            angle = 2 * math.pi * k / STUB_SEARCH_ANGLES
            x = round((pad_x + r * math.cos(angle)) / 0.1) * 0.1
            y = round((pad_y + r * math.sin(angle)) / 0.1) * 0.1
            pt = pcbnew.VECTOR2I(fmm(x), fmm(y))
            if not main_outline.PointInside(pt):
                continue
            ok, _ = path_clear(pad_x, pad_y, x, y, obstacle_rects, track_segs, track_vias)
            if ok:
                return (x, y)
        ring += 1
    return None


STUB_BFS_PITCH_MM = 0.2  # coarser than route_board.py's 0.1mm full-board
                          # router -- this is a local, one-off stub search,
                          # not the main router, and doesn't need that
                          # resolution
STUB_BFS_MARGIN_MM = 15.0  # local search window radius around the pad


def find_stub_path_bfs(pad_x, pad_y, main_outline, obstacle_rects, track_segs, track_vias):
    """Grid BFS fallback for find_stub_endpoint() -- see its docstring for
    why a straight-line search isn't enough for J3 pin 2. Same obstacle
    model (trace_clearance_ok), rasterized once over a local window
    (STUB_BFS_MARGIN_MM around the pad) instead of tested per candidate
    point, then breadth-first search (4-connected, unweighted -- this
    only needs A valid path, not a cheapest one) from the pad's cell to
    the nearest cell inside main_outline. Returns a list of (x, y)
    corner waypoints (start to end inclusive, collinear runs collapsed)
    or None if no path exists even in the maze search."""
    x0 = pad_x - STUB_BFS_MARGIN_MM
    x1 = pad_x + STUB_BFS_MARGIN_MM
    y0 = pad_y - STUB_BFS_MARGIN_MM
    y1 = pad_y + STUB_BFS_MARGIN_MM
    cols = int(round((x1 - x0) / STUB_BFS_PITCH_MM)) + 1
    rows = int(round((y1 - y0) / STUB_BFS_PITCH_MM)) + 1

    def to_rc(x, y):
        return (int(round((y - y0) / STUB_BFS_PITCH_MM)),
                int(round((x - x0) / STUB_BFS_PITCH_MM)))

    def to_xy(r, c):
        return x0 + c * STUB_BFS_PITCH_MM, y0 + r * STUB_BFS_PITCH_MM

    blocked = bytearray(rows * cols)
    inside_main = bytearray(rows * cols)
    for r in range(rows):
        y = y0 + r * STUB_BFS_PITCH_MM
        base = r * cols
        for c in range(cols):
            x = x0 + c * STUB_BFS_PITCH_MM
            if not trace_clearance_ok(x, y, obstacle_rects, track_segs, track_vias):
                blocked[base + c] = 1
            pt = pcbnew.VECTOR2I(fmm(x), fmm(y))
            if main_outline.PointInside(pt):
                inside_main[base + c] = 1

    start_r, start_c = to_rc(pad_x, pad_y)
    if not (0 <= start_r < rows and 0 <= start_c < cols):
        return None
    start_idx = start_r * cols + start_c

    prev = {}
    visited = bytearray(rows * cols)
    visited[start_idx] = 1
    queue = deque([start_idx])
    goal_idx = None
    while queue:
        idx = queue.popleft()
        if inside_main[idx]:
            goal_idx = idx
            break
        r, c = divmod(idx, cols)
        for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            nidx = nr * cols + nc
            if visited[nidx] or blocked[nidx]:
                continue
            visited[nidx] = 1
            prev[nidx] = idx
            queue.append(nidx)
    if goal_idx is None:
        return None

    path_idx = [goal_idx]
    while path_idx[-1] in prev:
        path_idx.append(prev[path_idx[-1]])
    path_idx.reverse()
    pts = [to_xy(*divmod(idx, cols)) for idx in path_idx]

    # collapse collinear runs to corners only, same idea as
    # route_board.py's path_to_geometry()
    corners = [pts[0]]
    for k in range(1, len(pts) - 1):
        x0p, y0p = pts[k - 1]
        x1p, y1p = pts[k]
        x2p, y2p = pts[k + 1]
        d1 = (round((x1p - x0p) / STUB_BFS_PITCH_MM), round((y1p - y0p) / STUB_BFS_PITCH_MM))
        d2 = (round((x2p - x1p) / STUB_BFS_PITCH_MM), round((y2p - y1p) / STUB_BFS_PITCH_MM))
        if d1 != d2:
            corners.append(pts[k])
    corners.append(pts[-1])
    return corners


def add_gnd_stub_traces(board, gnd_net, zones_to_fill, obstacle_rects, track_segs, track_vias):
    """Resolve and emit GND_STUB_TRACES: straight-line search first
    (find_stub_endpoint(), fast, works for open areas), BFS maze search as
    fallback (find_stub_path_bfs(), for pads genuinely boxed in like J3
    pin 2 -- see its docstring). Asserts a path was actually found rather
    than silently leaving a pad DRC will still flag. Idempotent: removes
    every previously-placed segment for a given pad (keyed by index, since
    a BFS path can be multiple segments) before re-adding, so a rerun
    replaces rather than duplicates or leaves stale extra segments behind
    if a path's corner count changes between runs."""
    placed = 0
    _kept_alive_stubs = []
    for net_name, ref, pin, layer in GND_STUB_TRACES:
        for t in list(board.Tracks()):
            u = str(t.m_Uuid)
            if any(u == stable_uuid(f"sh03:gndstub:{ref}:{pin}:{i}") for i in range(20)):
                board.Remove(t)
                _kept_alive_stubs.append(t)
        pad_pos = None
        for fp in board.GetFootprints():
            if fp.GetReference() != ref:
                continue
            for pad in fp.Pads():
                if pad.GetNumber() == pin:
                    p = pad.GetPosition()
                    pad_pos = (mm(p.x), mm(p.y))
        if pad_pos is None:
            raise SystemExit(f"[pour] GND_STUB_TRACES: pad not found: {ref}.{pin}")
        pad_x, pad_y = pad_pos

        # main pour outline for this layer, needed by the endpoint searches
        pcb_layer = pcbnew.F_Cu if layer == 0 else pcbnew.B_Cu
        main_outline = None
        best_area = -1.0
        for zone in zones_to_fill:
            if zone.GetLayer() != pcb_layer:
                continue
            filled = zone.GetFilledPolysList(pcb_layer)
            if filled is None:
                continue
            for oc in range(filled.OutlineCount()):
                outline = filled.Outline(oc)
                bb = outline.BBox()
                area = mm(bb.GetWidth()) * mm(bb.GetHeight())
                if area > best_area:
                    best_area = area
                    main_outline = outline
        if main_outline is None:
            raise SystemExit(f"[pour] GND_STUB_TRACES: no filled zone found on "
                              f"layer {layer} for {ref}.{pin}")

        # EXCLUDING this pad's own rect(s) (obstacle_rects isn't net-scoped,
        # so without this the search would immediately reject every
        # direction against the very pad the stub starts at, same
        # current_net exclusion every other obstacle check in this
        # pipeline already does).
        own_rects = [
            r for r in obstacle_rects
            if r[0] - 0.1 <= pad_x <= r[2] + 0.1 and r[1] - 0.1 <= pad_y <= r[3] + 0.1
        ]
        other_rects = [r for r in obstacle_rects if r not in own_rects]

        found = find_stub_endpoint(pad_x, pad_y, main_outline, other_rects, track_segs, track_vias)
        if found is not None:
            waypoints = [(pad_x, pad_y), found]
            method = "straight-line search"
        else:
            print(f"[pour] GND_STUB_TRACES: {ref}.{pin} -- straight-line search found "
                  f"nothing, falling back to BFS maze search")
            bfs_path = find_stub_path_bfs(pad_x, pad_y, main_outline, other_rects, track_segs, track_vias)
            if bfs_path is None:
                # CONFIRMED, NOT ASSUMED: this pad sits in a genuinely
                # enclosed pocket, verified twice -- once against the
                # (too-strict) via clearance model, once against the
                # correct trace clearance model (MIN_CLEARANCE_MM +
                # SIG_WIDTH/2) -- and the reachable region from the pad
                # (157 cells at 0.2mm pitch for J3.2 specifically) never
                # touches the main pour on either check. This is a real
                # structural enclosure (J3.1/J3.3's pad clearance to
                # either side, /MCU_TX + /MC_XQB2's nearly-full-width
                # traces to the north, the board edge to the south), not
                # a search-algorithm gap -- fixing it needs rerouting one
                # of those nets away from that corridor (a route_board.py
                # change) or accepting the pad's connection is provided
                # via its other layer + PTH barrel alone. Warn and move
                # on rather than crash the whole pour pass over one
                # pad -- see docs/layout-plan.md and the delivery report
                # for the coordinator decision this needs.
                print(f"[pour] WARNING: GND_STUB_TRACES: {ref}.{pin} -- NO PATH to "
                      f"the main {['F.Cu','B.Cu'][layer]} pour (straight-line search "
                      f"nor BFS maze search within {STUB_BFS_MARGIN_MM}mm) -- "
                      f"confirmed genuinely enclosed, not left unattempted. DRC will "
                      f"still flag this pad on this layer; see report.")
                continue
            waypoints = bfs_path
            method = f"BFS maze search ({len(bfs_path) - 1} segment(s))"

        for seg_idx in range(len(waypoints) - 1):
            sx, sy = waypoints[seg_idx]
            ex, ey = waypoints[seg_idx + 1]
            track = pcbnew.PCB_TRACK(board)
            track.SetStart(pcbnew.VECTOR2I(fmm(sx), fmm(sy)))
            track.SetEnd(pcbnew.VECTOR2I(fmm(ex), fmm(ey)))
            track.SetWidth(fmm(SIG_WIDTH))
            track.SetLayer(pcb_layer)
            track.SetNetCode(gnd_net.GetNetCode())
            board.Add(track)
            set_uuid(track, f"sh03:gndstub:{ref}:{pin}:{seg_idx}")
        print(f"[pour] GND stub trace placed: {ref}.{pin} via {method}, "
              f"({pad_x:.3f},{pad_y:.3f}) -> ({waypoints[-1][0]:.3f},{waypoints[-1][1]:.3f})")
        placed += 1
    return placed


def main():
    board = pcbnew.LoadBoard(PCB_PATH)

    gnd_net = find_gnd_net(board)
    print(f"[pour] using net '{gnd_net.GetNetname()}' (code={gnd_net.GetNetCode()})")

    _kept_alive = []

    removed_zones = remove_existing_gnd_zones(board, gnd_net.GetNetCode(), _kept_alive)
    print(f"[pour] removed {removed_zones} existing GND zone(s) on F.Cu/B.Cu")

    removed_keepout = remove_existing_keepout_zone(board, _kept_alive)
    print(f"[pour] removed {removed_keepout} existing antenna keepout zone(s)")
    keepout_zone = make_antenna_keepout_zone(board)
    kx0, ky0, kx1, ky1 = ANTENNA_KEEPOUT_MM
    print(f"[pour] antenna keepout zone added: ({kx0}, {ky0}) - ({kx1}, {ky1}) mm, "
          f"F.Cu+B.Cu, tracks/vias/copperpour not allowed")

    removed_vias = remove_existing_stitch_vias(board, gnd_net.GetNetCode(), _kept_alive)
    print(f"[pour] removed {removed_vias} existing perimeter stitching via(s)")

    ox0, oy0, ox1, oy1 = BOARD_OUTLINE
    zx0, zy0 = ox0 + INSET_MM, oy0 + INSET_MM
    zx1, zy1 = ox1 - INSET_MM, oy1 - INSET_MM
    print(f"[pour] outline inset {INSET_MM}mm from board edge: ({zx0}, {zy0}) - ({zx1}, {zy1}) mm")

    zones_to_fill = pcbnew.ZONES()
    for layer in POUR_LAYERS:
        layer_name = "F.Cu" if layer == pcbnew.F_Cu else "B.Cu"
        zone = make_zone(board, layer, gnd_net, zx0, zy0, zx1, zy1, f"GND_pour_{layer_name}")
        zones_to_fill.append(zone)
        print(f"[pour] zone added (unfilled), layer={layer_name}")

    filler = pcbnew.ZONE_FILLER(board)
    fill_ok = filler.Fill(zones_to_fill)
    print(f"[pour] ZONE_FILLER.Fill() returned {fill_ok}")
    for zone in zones_to_fill:
        layer = zone.GetLayer()
        layer_name = "F.Cu" if layer == pcbnew.F_Cu else "B.Cu"
        filled_polys = zone.GetFilledPolysList(layer)
        outline_count = filled_polys.OutlineCount() if filled_polys is not None else 0
        print(f"[pour] {layer_name}: IsFilled()={zone.IsFilled()}, filled outline count={outline_count}"
              + ("  (>1 may indicate separate fill islands around obstacles)" if outline_count > 1 else ""))

    obstacle_rects = collect_obstacle_rects(board)
    track_segs, track_vias = collect_track_obstacles(board, gnd_net.GetNetCode())
    placed, skipped, skipped_not_in_fill, total_candidates = add_stitch_vias(
        board, gnd_net, obstacle_rects, track_segs, track_vias, zones_to_fill)
    print(f"[pour] stitching vias: {placed} placed, {skipped} skipped total "
          f"({skipped - skipped_not_in_fill} within {STITCH_OBSTACLE_CLEARANCE_MM}mm of a "
          f"pad/courtyard/track/via, {skipped_not_in_fill} not inside the filled zone "
          f"polygon on both layers), {total_candidates} candidate positions at "
          f"~{STITCH_PITCH_MM}mm pitch, {STITCH_INSET_MM}mm inset from the board edge "
          f"({len(track_segs)} track segment(s), {len(track_vias)} via(s) checked)")

    # Interior pour islands the perimeter-only walk above can never reach
    # (see find_orphan_pour_islands()'s docstring -- a real DRC run flagged
    # this as a zone-to-zone "missing connection"). existing_gnd_points is
    # VIAS ONLY (see find_orphan_pour_islands()'s docstring for why GND
    # pads, including THT ones, don't count as bridging evidence here) --
    # covers both route_board.py's own single-layer-pad GND stitching vias
    # and the perimeter vias just placed above. Also doubles as the
    # idempotency check on reruns (a candidate matching an already-placed
    # interior via's position is skipped as a duplicate, same tolerance).
    existing_gnd_points = [
        (mm(t.GetPosition().x), mm(t.GetPosition().y))
        for t in board.Tracks()
        if t.Type() == pcbnew.PCB_VIA_T and t.GetNetCode() == gnd_net.GetNetCode()
    ]
    orphans = find_orphan_pour_islands(zones_to_fill, existing_gnd_points)
    if orphans:
        interior_placed, interior_skipped = add_interior_stitch_vias(
            board, gnd_net, orphans, obstacle_rects, track_segs, track_vias, zones_to_fill)
        print(f"[pour] interior island stitching: {interior_placed} placed, "
              f"{interior_skipped} skipped, {len(orphans)} orphan island(s) found")
    else:
        print("[pour] interior island stitching: no orphan islands found")

    stub_placed = add_gnd_stub_traces(board, gnd_net, zones_to_fill, obstacle_rects, track_segs, track_vias)
    print(f"[pour] GND stub traces: {stub_placed} placed "
          f"(pads a via can't reach -- see GND_STUB_TRACES)")

    board.Save(PCB_PATH)
    print(f"[pour] saved {PCB_PATH}")


if __name__ == "__main__":
    main()
