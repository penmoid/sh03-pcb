"""route_board.py - grid-based maze router for sh03-controller.kicad_pcb.

Run with KiCad's bundled interpreter:
  python3 scripts/route_board.py

Replaces route_signals.py (deleted): that script hand-specified a single
B.Cu lane/channel scheme and got stuck because the J1/J2-to-U1 pin
permutation is provably non-planar on one copper layer (see its old
docstring in git history). This script instead does a real two-layer A*
maze router over a 0.1mm grid, so layer changes (vias) are just another
priced move instead of something to avoid.

Idempotent: deletes every PCB_TRACK/PCB_VIA on the board on each run, then
regenerates from scratch. Pad positions and nets are read live from the
board. Run this BEFORE scripts/add_ground_pour.py (same ordering rule as
the old script -- the pour's perimeter stitching vias must be added last).

Determinism
------------
Every ordering decision below is total and explicit -- nothing is left to
rely on dict/set/container iteration order:
  - collect_pads() walks footprints and pads in refdes/pad-number sorted
    order, not board.GetFootprints() order. That matters: pcbnew's
    footprint container order after LoadBoard() is itself a function of
    the footprints' UUID values (verified empirically against KiCad
    10.0.5 -- two boards differing only in footprint UUID text produced
    different board.GetFootprints() order after a save+reload). With
    place_footprints.py now minting deterministic UUIDs this is already
    fixed upstream, but sorting explicitly here means route_board.py's
    output no longer depends on that pcbnew implementation detail at all.
  - Net routing order sorts by (mst_length, net_name) -- name is the final
    tie-break so exact-length ties never fall back to insertion order.
  - Rip-up "blockers" lists are sorted by name before being requeued.
  - A*'s heap already carried a monotonic push counter as a tiebreaker
    (unchanged) -- (cost, counter, node_id) is a total order with no ties
    possible, so equal-cost frontier expansion was already deterministic;
    verified rather than modified.
  - Every PCB_TRACK/PCB_VIA this script creates gets a UUID5 derived from
    (net name, item kind, index within that net's routed geometry) instead
    of pcbnew's random default, so a byte-identical routing outcome also
    serializes to byte-identical file content.

Algorithm summary
------------------
- Uniform grid, 0.1mm pitch, over the board outline, two layers (F.Cu/B.Cu).
- Per net, an obstacle grid is rasterized fresh from: board edge keepout,
  mounting-hole keepout, pads of every OTHER net, and tracks/vias of every
  OTHER currently-active net -- each expanded by clearance + the current
  net's own half trace-width, so wider nets (the two power rails) get
  proportionally more berth.
- Each net is routed as a minimum spanning tree over its pads (Prim's).
  Edges are routed in tree-growth order: A* from the newly-joined pad to
  the set of every grid cell already claimed by this net's tree (any point
  on an already-drawn segment, not just pads), so branches join naturally.
- A* is 8-connected with a small turn penalty, plus a via move (cost 12)
  between the same (row, col) on the two layers. Through-hole pads (U1,
  J3) sit in both layers already -- crossing layers exactly at one of
  those pads' grid cells is free (it's the same physical plated hole, not
  a via).
- If a net's MST can't be fully routed, rip up nearby already-routed nets
  and retry; ripped nets go back on the queue. Capped iteration count.

GND is not routed here -- it's the pour in add_ground_pour.py. This script
only drops a stitching via next to any GND pad that lives on a single
copper layer, so both pours pick it up.
"""

import heapq
import math
import os
import sys
import time
import uuid
from collections import deque

import pcbnew

_T0 = time.time()


def log(msg):
    print(f"[route] t={time.time() - _T0:7.1f}s {msg}", flush=True)

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


# --- board / grid geometry -------------------------------------------------

X0, Y0, X1, Y1 = 36.0, 33.0, 136.0, 89.0
# East edge widened 132 -> 136 (96 -> 100mm board width, 2026-08-12d
# east-widen rework, kept under JLCPCB's <=100x100mm price tier) -- see
# docs/layout-plan.md. The new strip (X 132-136) is empty margin, so the
# routing grid simply grows by 40 columns; no other constant here depends
# on the old width.
PITCH = 0.1
COLS = round((X1 - X0) / PITCH) + 1
ROWS = round((Y1 - Y0) / PITCH) + 1
CELLS_PER_LAYER = ROWS * COLS
NUM_CELLS = 2 * CELLS_PER_LAYER
LAYER_NAMES = ["F.Cu", "B.Cu"]
LAYER_IDS = [pcbnew.F_Cu, pcbnew.B_Cu]

# Board setup's copper-to-edge design rule (confirmed against
# reports/drc-after.json's "board setup constraints edge clearance 0.5000
# mm" message -- keep this in sync with that value). This used to be a flat
# 0.3mm keepout applied identically to every net regardless of trace width,
# which under-provisioned even ordinary signal traces (0.3mm keepout - a
# via-pad-radius-dominated 0.3mm half_width left 0mm of guaranteed copper-
# to-edge clearance in the worst case) and produced real DRC
# copper_edge_clearance violations for the 1.0mm-wide /+5V net once routing
# actually reached the top edge. Fixed by making the edge (and mounting
# hole) keepout scale with the routing net's own half_width, the same
# CLEARANCE+half_width pattern build_blocked_for_net already uses for pads
# and tracks -- see build_static_blocked() and get_static_blocked() below.
BOARD_EDGE_CLEARANCE_MM = 0.5
MOUNTING_HOLE_KEEPOUT = 0.5  # added to the hole's own drill radius

CLEARANCE = 0.2

SIG_WIDTH = 0.25
V3V3_WIDTH = 0.4
V5V_WIDTH = 1.0

VIA_DRILL = 0.3
VIA_PAD = 0.6
VIA_COST = 12.0
TURN_PENALTY = 0.3
DIAG = math.sqrt(2)

MOUNTING_HOLE_REFS = ["H1", "H2", "H3", "H4"]

# RF antenna keepout under U1's DevKitC-1 module (pin-1 / west end -- see
# add_ground_pour.py's ANTENNA_KEEPOUT_MM for the full derivation, mirrored
# here so the router treats the same rectangle as a hard obstacle on both
# copper layers, for every net, same as the board-edge/mounting-hole static
# keepouts above. Pads and footprints are NOT excluded by this (this grid
# only governs where route_board.py may lay copper) -- U1's own footprint
# still sits here untouched, matching the (pads allowed)(footprints allowed)
# rule area emitted by add_ground_pour.py.
#
# Y is narrowed to the INTER-ROW strip (between U1's two 22-pin header
# rows), not the module's full courtyard Y-span -- the antenna itself runs
# down the module's center, between the rows; the pin rows are where every
# DevKitC carrier necessarily routes and must stay clear. A full-Y-span
# keepout blocked U1's own pin-1/pin-2 pads (both /+3V3, both inside the
# X-reach), making that net's MST edge R2.1->U1.1 structurally unroutable
# (A* found zero valid start nodes on either layer, not a search failure --
# verified via a real route_board.py run before this narrowing).
ANTENNA_KEEPOUT_MM = (65.635, 52.17, 75.635, 70.03)  # (x0, y0, x1, y1)

# Radius (mm) around a failed net's own pads used to decide which
# currently-active nets count as "blockers" to rip up and retry. Was 15mm.
# Diagnosed via a timed, timeout-bounded route_board.py run with per-net
# progress logging (net name, attempt#, elapsed, active_nets count) added:
# individual A* solves were fast (0.0-0.5s each), but active_nets oscillated
# between 0 and ~20/29 over hundreds of iterations, never converging --
# classic rip-up/retry livelock, NOT board congestion or an unroutable net.
# Root cause: the board is only 96x56mm, so a +-15mm pad turns fail_bbox
# into a 30x30mm box -- roughly half the board on a side -- and most nets'
# own routed bboxes (e.g. a TP-to-J1-to-U1 run) already span a large
# fraction of the board, so nearly every currently-active net counts as a
# "blocker" on nearly every single failure (verified: 18 rip-up events
# logged in one ~250s window, average 13.5 blockers ripped per event out of
# a max of 29 total nets). Each failure was wiping out most of the board's
# progress and starting over, which is why it never terminated.
# Shrunk to 3mm: still generous relative to trace width (0.25mm) and via
# pad (0.6mm), but tight relative to board scale, so a failure only rips up
# nets that are genuinely nearby in the region that blocked it. The tier-2
# fallback (rip up every active net) below is unchanged and still fires if
# 3mm finds no blockers, so a genuinely wide-ranging conflict still gets a
# full-board retry -- this only removes the case where EVERY failure
# defaulted to a near-full-board rip-up regardless of where the conflict
# actually was.
RIPUP_MARGIN_MM = 3.0
# TRIED, REVERTED (2026-08-11 TP-removal livelock investigation, lever 2 of
# 3): bumping this to 5.0mm was tried after PRIORITY_NETS (lever 1) alone
# left /+3V3 (and, once priority routed first, intermittently /MC_HOT1)
# cycling a fixed blocker set with zero progress for 5+ minutes. It had NO
# measurable effect -- the resulting run log was byte-for-byte the same
# failure signature (same net, same blocker list, same ~7s cadence) as
# lever 1 alone, confirmed by diffing both runs' logs. The failing edges' own pad-to-pad span (R2.1->U1.1: 27-29mm on a
# 96x56mm board) already produces a fail_bbox large enough to pull in most
# of the board's other nets regardless of a couple more mm of margin, so
# this lever never had room to change the outcome. See docs/layout-plan.md's
# TP-removal section for the full lever 1/2/3 sequence and its outcome.
# Left at its original 3.0mm.

RIP_UP_ITERATION_CAP = 12000

# Nets whose R1-R4 <-> U1 trunk edge deterministically livelocks now that
# TP1-29 are gone. Those TPs were PTH pads sitting ON these exact nets
# (TP2 on /TH1, TP1 on /TH2, etc. -- see git history / place_footprints.py
# before the 2026-08-11 TP removal) -- the old router could treat a TP as a
# free layer-change point and an MST intermediate target, splitting what is
# now one long R-to-U1 edge into two short ones through the R1-R4<->U1
# corridor. With the TPs gone, /+3V3, /TH1 and /TH2 each collapse to a
# single long edge contesting that corridor, and the rip-up/retry queue
# (which has no progress/history cost -- see ripup_route_phase()) can cycle
# a fixed blocker set forever instead of converging: confirmed via a real
# route_board.py run, periodic to the iteration (same net, same blocker
# set, repeating every ~27 iterations, 14+ consecutive cycles with zero
# progress) both WITH the antenna keepout in place (/+3V3 then /TH1 stuck)
# and with it removed entirely (/TH2 stuck instead) -- so the antenna
# keepout narrows the corridor further but is not the root cause; the
# corridor itself lost capacity when the TPs left. Also confirmed these
# edges route trivially (well under a second) when given the whole board
# to themselves, i.e. this is contention/ordering, not infeasibility.
#
# Fix: route these first, in this fixed order, ahead of every other net
# (including the safety/power tiers below) -- while J1/J2's escape lanes
# and the R1-R4<->U1 corridor are still completely empty, so they claim it
# instead of relying on rip-up/retry to win it later. An explicit named
# list, not a heuristic (e.g. "nets touching R1-R4"), so the ordering stays
# deterministic and auditable independent of any future net additions.
PRIORITY_NETS = ("/TH2", "/TH1", "/+3V3")
# Experiment A (2026-08-11, per coordinator): TH2 has the smallest goal set
# (single stub tree), /+3V3 has the largest (multi-target, several pads)
# and already proved it can find long alternates -- so give the most
# constrained net first pick of the Y[45.5,48.0] locked-stub highway (see
# this file's earlier attribution-map investigation) and let the most
# flexible net scavenge whatever's left. Was ("/+3V3", "/TH1", "/TH2").

# TRIED, REVERTED (2026-08-11 TP-removal livelock investigation, candidate
# 2 of 2 authorized after PRIORITY_NETS/RIPUP_MARGIN_MM/R1-R4-shift and a
# multi-target A* start -- see route_edges() -- all failed or proved
# insufficient alone): exempting a small, explicit set of nets' phase-1
# escape stubs from the normal permanent lock (see ripup_route_phase()'s
# locked_active docstring), so rip-up could revisit them like ordinary
# trunk geometry instead of treating them as permanent fixtures.
#
# Diagnosis (direct reproduction against route_board.py's own functions,
# not just the full rip-up driver, each step re-verified rather than
# assumed): with every OTHER net's phase-1 stub present and everything else
# cleared -- the absolute floor tier-2 rip-up can never get past -- /+3V3's
# R2.1->U1.1, /TH1's R2.2->U1.27 and /TH2's R1.2->U1.26 all still returned
# NO PATH, while a genuinely empty board (zero other nets, not even stubs)
# routed all three fine. Removing ANY ONE of the 27 other nets' stubs
# independently unblocked all three -- a collective-congestion effect, not
# one identifiable wall, so there was never a single "the blocking net" to
# name.
#
# Exempting just /MC_HALLA (shortest MST trunk among the 27 working
# candidates, target pin immediately at the antenna keepout's east edge)
# fixed /+3V3 and /MC_HOT1//MC_HOT2 cleanly and repeatably in live runs, but
# left /TH1 livelocking (a real, non-terminating rip-up cycle -- 18
# blockers found and ripped every time, never converging) and /TH2 failing
# cleanly-but-permanently (zero blockers found at tier2, so it hit the
# router's own "FAILED -- no path even with every rippable net ripped up"
# terminal state after 2 attempts). Re-diagnosing against a REALISTIC
# obstacle set (every other net actually routed to a settled state, not
# just the bare locked-stub floor) found /MC_XQD2 and /MC_XQA1 (both
# north-row, adjacent to /TH1's and /TH2's own targets) as additional
# single-stub relief for /TH1 -- but adding them on top of /MC_HALLA in a
# live run caused a NEW regression: /+3V3 (previously fixed) and /TH2 both
# started cycling instead. Freeing more of the shared corridor didn't add
# capacity, it just relocated the same contention -- classic whack-a-mole,
# not a fix converging toward a stable state. Reverted entirely rather than
# chase a 4th, 5th exemption; see docs/layout-plan.md for the full
# candidate 1/2 sequence and where it was left for the coordinator.
STUB_UNLOCK_NETS = ()

# --- pre-routed segments (deterministic, locked before phase 1 even starts) -
#
# 2026-08-11: PRIORITY_NETS reordering (Experiment A), SOFT_COST_REGIONS
# (a real fix for the fan-net highway, kept), and every other router-side
# mitigation tried still couldn't get /TH1 and /TH2 to both complete --
# they're two nets whose short final hop (R pad -> adjacent U1 pin) sit
# 2.54mm apart, and whichever one's A* search runs first claims the other's
# channel, no matter the cost tuning. Two interleaved nets fighting over
# immediately-adjacent slots has no stable cost-tuned resolution -- so stop
# negotiating and pre-route both hops as straight, deterministic tracks
# before any A* search (this net's own or any other net's) ever runs.
#
# Locked bookkeeping matches phase-1 stub geometry exactly (see
# ripup_route_phase()'s locked_active docstring): once resolved, these
# segments are visible to every OTHER net's obstacle grid from the very
# first phase-1 iteration (via _PRE_ROUTE_SEGMENTS in
# build_blocked_for_net(), independent of the active_nets/locked_active
# rip-up bookkeeping entirely, so rip-up can never touch them at any point
# in either phase) and are spliced into the OWNING net's own tree_cells
# right after phase 1 completes (see splice_pre_route_into_result()) so the
# MST edge connecting the pre-routed pad degenerates into the same
# already-in-tree no-op the multi-target lookahead already produces for
# /+3V3's U1.1<->U1.2 hop -- no new short-circuit logic needed, just correct
# seeding. NOT spliced in before phase 1's OWN escape-stub edge (J1->U1.26/
# 27) runs, deliberately: that edge still needs to do real work connecting
# to J1, or the net would end up with a pre-routed R<->U1 pair floating
# disconnected from the rest of the board.
PRE_ROUTES = [
    # (net_name, from_ref, from_pin, to_ref, to_pin, layer_index)
    # Layer 1 = B.Cu (LAYER_IDS[1]) -- moved from F.Cu 2026-08-12 when R1/R2
    # joined the rest of the SMD population on the back (all-SMD-on-back for
    # single-side JLCPCB Economic PCBA; see place_footprints.py's PLACEMENT
    # comment). U1 is a THT socket (PTH barrels, copper on both layers), so
    # the B.Cu endpoint is exactly as valid a target as F.Cu was -- nothing
    # else about this pre-route changes. Straight vertical -- resolve_pre_routes()
    # asserts the pad pair is X-aligned within 0.05mm (they were placed
    # aligned by construction, see place_footprints.py's PLACEMENT table)
    # and that every cell along the segment is clear on the static
    # (pad-free) obstacle grid, rather than trusting either assumption.
    ("/TH2", "R1", "2", "U1", "26", 1),
    ("/TH1", "R2", "2", "U1", "27", 1),
]

_PRE_ROUTE_SEGMENTS = []  # resolved by resolve_pre_routes(), read by build_blocked_for_net()


def resolve_pre_routes(all_pads):
    """Resolve PRE_ROUTES against live pad positions (not hardcoded
    coordinates) and fail loudly rather than silently emitting a dogleg or
    routing through an obstacle. Returns a list of dicts: net, layer, x,
    y0, y1, width, from, to."""
    pad_lookup = {(p["ref"], p["pin"]): p for p in all_pads}
    resolved = []
    for net_name, from_ref, from_pin, to_ref, to_pin, layer in PRE_ROUTES:
        a = pad_lookup.get((from_ref, from_pin))
        b = pad_lookup.get((to_ref, to_pin))
        if a is None or b is None:
            raise SystemExit(
                f"[route] PRE_ROUTES: pad not found for {net_name} "
                f"({from_ref}.{from_pin} or {to_ref}.{to_pin})")
        if a["net"] != net_name or b["net"] != net_name:
            raise SystemExit(
                f"[route] PRE_ROUTES: {from_ref}.{from_pin}/{to_ref}.{to_pin} not "
                f"both on {net_name} (got {a['net']!r}/{b['net']!r}) -- netlist "
                f"or placement changed under this pipeline stage")
        dx = abs(a["x"] - b["x"])
        if dx >= 0.05:
            raise SystemExit(
                f"[route] PRE_ROUTES: {net_name} {from_ref}.{from_pin}->"
                f"{to_ref}.{to_pin} X misaligned by {dx:.4f}mm (>=0.05mm) -- "
                f"placement drifted out from under this pipeline stage, "
                f"refusing to emit a dogleg")
        x = (a["x"] + b["x"]) / 2.0
        y0, y1 = sorted((a["y"], b["y"]))
        width = SIG_WIDTH
        half_width = max(width / 2.0, VIA_PAD / 2.0)
        static_blocked = get_static_blocked(half_width)
        r0 = round((y0 - Y0) / PITCH)
        r1 = round((y1 - Y0) / PITCH)
        col = round((x - X0) / PITCH)
        for row in range(r0, r1 + 1):
            n = nid(layer, row, col)
            if static_blocked[n]:
                raise SystemExit(
                    f"[route] PRE_ROUTES: {net_name} segment at X={x:.3f} "
                    f"Y={Y0 + row * PITCH:.3f} layer={layer} is statically "
                    f"blocked (edge margin / mounting hole / antenna keepout) "
                    f"on the empty board -- placement drifted, refusing to "
                    f"route through an obstacle")
        resolved.append({
            "net": net_name, "layer": layer, "x": x, "y0": y0, "y1": y1,
            "width": width, "from": (from_ref, from_pin), "to": (to_ref, to_pin),
        })
        print(f"[route] PRE_ROUTE resolved: {net_name} {from_ref}.{from_pin}->"
              f"{to_ref}.{to_pin} X={x:.3f} Y[{y0:.3f},{y1:.3f}] layer={layer}",
              flush=True)
    return resolved


def splice_pre_route_into_result(result, pr):
    """Splice a resolved pre-route into a net's phase-1 result AFTER that
    net's own escape-stub edge has genuinely connected it to J1/J2 -- see
    PRE_ROUTES' module comment for why this must happen after, not before,
    phase 1's own edge for this net."""
    result = dict(result)
    result["segments"] = list(result["segments"]) + [
        (pr["layer"], pr["x"], pr["y0"], pr["x"], pr["y1"])
    ]
    tree_cells = set(result["tree_cells"])
    r0 = round((pr["y0"] - Y0) / PITCH)
    r1 = round((pr["y1"] - Y0) / PITCH)
    col = round((pr["x"] - X0) / PITCH)
    for row in range(r0, r1 + 1):
        tree_cells.add(nid(pr["layer"], row, col))
    result["tree_cells"] = tree_cells
    result["edge_results"] = list(result["edge_results"]) + [{
        "from": pr["from"], "to": pr["to"], "length_mm": pr["y1"] - pr["y0"],
    }]
    return result


# --- per-net phase-1 edge selection override --------------------------
#
# 2026-08-11: /+3V3's regression after the antenna-keepout and multi-target
# orphan fixes (both correct, both kept) had the same shape as /TH1//TH2's
# original problem: phase 1 only ever locks in a net's FIRST MST edge
# (root -> nearest pad) as its escape stub, on the theory that's always the
# short, easy connector-escape hop -- true for every other net, but for
# /+3V3 the MST's first edge is the trivial 2.54mm R1.1-R2.1 tie, while the
# genuinely hard part (the ~31mm span from the R-cluster across to
# U1.1/U1.2, right at the grown antenna keepout's edge) is edge 2, left
# exposed to full phase-2 contention. Confirmed via direct reproduction:
# that exact edge finds a path fine (38.5mm) against the grown keepout on
# an empty board -- it's not structurally blocked, it's contested.
#
# Fix: for listed nets, phase 1 locks edges[0 : longest_edge_index+1]
# instead of just edges[0:1] -- i.e. every edge up to and including
# whichever one has the largest mst_dist, not just the first. This still
# respects Prim's ordering (nothing here reorders the edges themselves,
# it only widens phase 1's slice), so every edge's own "already in tree"
# precondition still holds normally. Phase 2 picks up wherever phase 1
# left off. The multi-target lookahead (see route_edges()'s alt_pads,
# extended in the same pass to search the net's FULL edge list rather than
# just the current call's edge_hi) still applies across the phase
# boundary, so the long edge gets the same alt-pad flexibility it would
# have had if it were still in phase 2 -- it just gets to use that
# flexibility while the board is still sparse instead of fully contested.
PHASE1_LONGEST_EDGE_NETS = ("/+3V3", "/+5V")


def phase1_edge_end(net_name, mst_cache):
    """Exclusive end index of this net's phase-1 edge slice: 1 (just the
    first/shortest edge) normally, or through the longest edge (inclusive)
    for nets in PHASE1_LONGEST_EDGE_NETS."""
    order, edges = mst_cache[net_name]
    if net_name in PHASE1_LONGEST_EDGE_NETS and len(edges) > 1:
        longest_idx = max(range(len(edges)), key=lambda idx: edges[idx][2])
        return longest_idx + 1
    return 1


# Empirically all 29 nets on this board route successfully in isolation
# (verified via a standalone per-net check against an empty active_nets
# set), so the original 300-iteration cap wasn't a real infeasibility --
# it was a rip-up/retry livelock: requeuing a just-failed net at the
# FRONT of the queue (queue.appendleft below) retries it immediately
# against a nearly-identical obstacle set, so it fails again and
# re-triggers the same rip-up, cycling the same few nets in and out of
# active_nets without making room elsewhere on the board. Requeuing at
# the BACK instead (see the two queue.append(net_name) calls below,
# changed from appendleft) forces a full round of every other queued net
# to get a turn before a repeat attempt -- the standard fix for this
# rip-up-and-retry livelock pattern. Verified empirically: the appendleft
# version still only routed 7/29 nets after 5000 iterations (accumulating
# 200+ repeat attempts on the same handful of nets); the append version
# reached 11/29 simultaneously active within 1500 iterations on a
# monotonically improving trajectory. The cap is raised accordingly to
# give the now-converging schedule enough iterations to finish.


def fmm(v):
    return pcbnew.FromMM(v)


def mm(nm):
    return pcbnew.ToMM(nm)


def net_width(name):
    if name == "/+5V":
        return V5V_WIDTH
    if name == "/+3V3":
        return V3V3_WIDTH
    return SIG_WIDTH


# --- grid <-> mm helpers -----------------------------------------------

def to_grid(x, y):
    col = round((x - X0) / PITCH)
    row = round((y - Y0) / PITCH)
    col = max(0, min(COLS - 1, col))
    row = max(0, min(ROWS - 1, row))
    return row, col


def to_mm(row, col):
    return X0 + col * PITCH, Y0 + row * PITCH


def nid(layer, row, col):
    return layer * CELLS_PER_LAYER + row * COLS + col


# --- board data extraction ----------------------------------------------

def pad_layers(pad):
    attr = pad.GetAttribute()
    if attr in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
        return [0, 1]
    ls = pad.GetLayerSet()
    layers = []
    if ls.Contains(pcbnew.F_Cu):
        layers.append(0)
    if ls.Contains(pcbnew.B_Cu):
        layers.append(1)
    return layers


def pad_bbox_mm(pad):
    bb = pad.GetBoundingBox()
    return mm(bb.GetLeft()), mm(bb.GetTop()), mm(bb.GetRight()), mm(bb.GetBottom())


def collect_pads(board):
    """Every pad on the board as a dict: ref, pin, net, x, y, bbox, layers,
    is_th. Walked in explicit (refdes, pad number) sorted order -- NOT
    board.GetFootprints()/fp.Pads() iteration order, which pcbnew derives
    in a way that's sensitive to footprint UUID values on a loaded board
    (see module docstring)."""
    pads = []
    for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
        ref = fp.GetReference()
        for pad in sorted(fp.Pads(), key=lambda p: p.GetNumber()):
            pos = pad.GetPosition()
            attr = pad.GetAttribute()
            is_th = attr in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
            pads.append({
                "ref": ref,
                "pin": pad.GetNumber(),
                "net": pad.GetNetname(),
                "x": mm(pos.x),
                "y": mm(pos.y),
                "bbox": pad_bbox_mm(pad),
                "layers": pad_layers(pad),
                "is_th": is_th,
            })
    return pads


def group_nets(pads):
    """net name -> list of pad dicts, for real routable multi-pad signal nets
    (excludes GND, no-net pads, and single-pad 'unconnected-*' nets). Pads
    within a net are explicitly sorted (ref, pin) -- collect_pads() already
    yields them in that order, but sorting again here makes the invariant
    self-evident and independent of caller order."""
    nets = {}
    for p in pads:
        name = p["net"]
        if not name or name == "/GND" or name.startswith("unconnected-"):
            continue
        nets.setdefault(name, []).append(p)
    return {
        name: sorted(pl, key=lambda p: (p["ref"], p["pin"]))
        for name, pl in nets.items()
        if len(pl) >= 2
    }


def get_pad(board, ref, pin):
    for fp in board.GetFootprints():
        if fp.GetReference() == ref:
            for pad in fp.Pads():
                if pad.GetNumber() == pin:
                    return pad
    raise SystemExit(f"[route] pad not found: {ref}.{pin}")


def get_net(board, name):
    ni = board.FindNet(name)
    if ni is None:
        raise SystemExit(f"[route] net not found: {name}")
    return ni


# --- rasterization --------------------------------------------------------

def clip_range(lo, hi, max_idx):
    lo = max(0, lo)
    hi = min(max_idx - 1, hi)
    return lo, hi


def rasterize_rect_mm(blocked, x0, y0, x1, y1, layers):
    c0 = math.floor((x0 - X0) / PITCH)
    c1 = math.ceil((x1 - X0) / PITCH)
    r0 = math.floor((y0 - Y0) / PITCH)
    r1 = math.ceil((y1 - Y0) / PITCH)
    r0, r1 = clip_range(r0, r1, ROWS)
    c0, c1 = clip_range(c0, c1, COLS)
    for layer in layers:
        base = layer * CELLS_PER_LAYER
        for row in range(r0, r1 + 1):
            off = base + row * COLS
            for col in range(c0, c1 + 1):
                blocked[off + col] = 1


def rasterize_circle_mm(blocked, cx, cy, radius, layers):
    c0 = math.floor((cx - radius - X0) / PITCH)
    c1 = math.ceil((cx + radius - X0) / PITCH)
    r0 = math.floor((cy - radius - Y0) / PITCH)
    r1 = math.ceil((cy + radius - Y0) / PITCH)
    r0, r1 = clip_range(r0, r1, ROWS)
    c0, c1 = clip_range(c0, c1, COLS)
    r2 = radius * radius
    for layer in layers:
        base = layer * CELLS_PER_LAYER
        for row in range(r0, r1 + 1):
            y = Y0 + row * PITCH
            dy2 = (y - cy) ** 2
            off = base + row * COLS
            for col in range(c0, c1 + 1):
                x = X0 + col * PITCH
                if (x - cx) ** 2 + dy2 <= r2:
                    blocked[off + col] = 1


def rasterize_segment_mm(blocked, layer, x0, y0, x1, y1, half_width):
    pad = half_width
    lo_x, hi_x = min(x0, x1) - pad, max(x0, x1) + pad
    lo_y, hi_y = min(y0, y1) - pad, max(y0, y1) + pad
    c0 = math.floor((lo_x - X0) / PITCH)
    c1 = math.ceil((hi_x - X0) / PITCH)
    r0 = math.floor((lo_y - Y0) / PITCH)
    r1 = math.ceil((hi_y - Y0) / PITCH)
    r0, r1 = clip_range(r0, r1, ROWS)
    c0, c1 = clip_range(c0, c1, COLS)
    dx, dy = x1 - x0, y1 - y0
    seg_len2 = dx * dx + dy * dy
    base = layer * CELLS_PER_LAYER
    hw2 = half_width * half_width
    for row in range(r0, r1 + 1):
        y = Y0 + row * PITCH
        off = base + row * COLS
        for col in range(c0, c1 + 1):
            x = X0 + col * PITCH
            if seg_len2 < 1e-9:
                d2 = (x - x0) ** 2 + (y - y0) ** 2
            else:
                t = ((x - x0) * dx + (y - y0) * dy) / seg_len2
                t = max(0.0, min(1.0, t))
                px, py = x0 + t * dx, y0 + t * dy
                d2 = (x - px) ** 2 + (y - py) ** 2
            if d2 <= hw2:
                blocked[off + col] = 1


# --- static obstacles (board edge + mounting holes) -----------------------

def build_static_blocked(mounting_holes, half_width):
    """half_width is the CURRENT net's own half trace-width (or via-pad
    radius, whichever is larger -- see route_edges()): both the edge
    keepout and the mounting-hole keepout must grow with it, or a wide net
    (only /+5V today) can have its centerline pass the design-rule check
    while its actual copper does not. See BOARD_EDGE_CLEARANCE_MM's
    comment for the DRC violation this fixes."""
    blocked = bytearray(NUM_CELLS)
    # board edge keepout
    edge_margin = BOARD_EDGE_CLEARANCE_MM + half_width
    for layer in (0, 1):
        base = layer * CELLS_PER_LAYER
        edge_cells = math.ceil(edge_margin / PITCH)
        for row in range(ROWS):
            off = base + row * COLS
            near_top_bottom = row < edge_cells or row >= ROWS - edge_cells
            if near_top_bottom:
                for col in range(COLS):
                    blocked[off + col] = 1
            else:
                for col in range(0, edge_cells):
                    blocked[off + col] = 1
                for col in range(COLS - edge_cells, COLS):
                    blocked[off + col] = 1
    # mounting holes
    for (cx, cy, drill) in mounting_holes:
        radius = drill / 2.0 + MOUNTING_HOLE_KEEPOUT + half_width
        rasterize_circle_mm(blocked, cx, cy, radius, (0, 1))
    # RF antenna keepout (see ANTENNA_KEEPOUT_MM). BUG FIXED 2026-08-11:
    # this used to be a flat block with no half_width growth, on the theory
    # that a hard rule-area boundary doesn't scale with the current net's
    # trace width the way the edge/hole keepouts do -- wrong in practice.
    # A real DRC run caught it: two vias (their CENTER outside the
    # rectangle, same as every other obstacle here, blocked cells are
    # tested by centerline) had their physical PAD extend into the keepout
    # ("items_not_allowed (keepout area 'U1_antenna_keepout')", 2 error-
    # severity violations), because the router only ever kept the via's
    # *center* out, not its copper. Same fix as the edge/hole keepouts:
    # grow the rectangle by the current net's own half_width (which is
    # already max(trace_half_width, VIA_PAD/2), so it inherently covers a
    # via's own radius too) before rasterizing.
    kx0, ky0, kx1, ky1 = ANTENNA_KEEPOUT_MM
    rasterize_rect_mm(blocked, kx0 - half_width, ky0 - half_width,
                       kx1 + half_width, ky1 + half_width, (0, 1))
    return blocked


# Cache of build_static_blocked() results keyed by half_width, since it's
# now net-dependent instead of a single board-wide constant -- rebuilding
# the full edge-keepout rasterization (O(ROWS*COLS)) for every one of the
# ~60 route_edges() calls in a run would be wasteful when in practice only
# 2 distinct half_width values occur (0.3mm for every net whose via-pad
# radius dominates its own trace half-width, 0.5mm for /+5V). Keyed off
# _MOUNTING_HOLES, set once by main() before any routing starts and never
# mutated afterward within a run.
_MOUNTING_HOLES = []
_static_blocked_cache = {}


def get_static_blocked(half_width):
    key = round(half_width, 6)
    cached = _static_blocked_cache.get(key)
    if cached is None:
        cached = build_static_blocked(_MOUNTING_HOLES, half_width)
        _static_blocked_cache[key] = cached
    return cached


# --- soft-cost regions (A* cost penalty, NOT an obstacle) ------------------
#
# 2026-08-11: the crossing-inventory investigation (see PRIORITY_NETS'
# comment and docs/layout-plan.md) found that Y[45.5, 48.0] -- the strip
# between R1/R2's row and U1's north pin row -- isn't blocked by many nets
# each crossing it once; it's occupied almost wall-to-wall (X[42.7, 124.5]
# on F.Cu, similar on B.Cu) by just 5 long-haul fan/humidity nets
# (/MC_K_FAN1, /MC_K_FAN1_FG, /MC_K_FAN2_FG, /MC_K_FAN3, /MC_K_FAN3_FG, plus
# /MC_HUM_SDA/SCL near the edges) that route roughly HORIZONTALLY through
# the strip for 60-135mm, because it's their cheapest unweighted path from
# J1 to U1 pins far to the east -- not because they need to be there. That
# leaves zero usable crossing gaps anywhere near R1/R2 (nearest gap is
# >30mm away at either board edge), which is what actually starved /TH1
# and /TH2's short local crossings.
#
# Fix: tax lingering in the strip instead of blocking it. A dict/list of
# (x0, y0, x1, y1, per_cell_penalty) rectangles, rasterized onto BOTH
# copper layers same as the hard obstacles above, but added as an
# ADDITIVE cost on top of the base per-cell step cost in astar() rather
# than a block -- every cell stays traversable, so a short perpendicular
# crossing barely notices it, but a long run spent INSIDE the region racks
# up the penalty once per cell traveled, same mechanism as TURN_PENALTY.
SOFT_COST_REGIONS = [
    # Region = the strip between R1/R2's southmost pad and U1's north row,
    # narrowed by ~0.3mm standoff from each side's pad halo (pad bounding
    # box + the router's own CLEARANCE constant, not the courtyard, since
    # that's the margin build_blocked_for_net() actually enforces):
    #   North edge: R1/R2's southmost pad (pin 2, the /TH1//TH2-side pad)
    #     bbox bottom = Y 44.425 (read off the live board). Halo =
    #     44.425 + CLEARANCE(0.2) = 44.625. +0.3mm standoff = 44.925.
    #   South edge: U1's north-row pads (pins 23-44, all Y=49.670, size
    #     1.7x1.7mm) bbox top = Y 48.820 (read off the live board). Halo =
    #     48.820 - CLEARANCE(0.2) = 48.620. -0.3mm standoff = 48.320.
    #   -> Y[44.925, 48.320], 3.395mm tall -- comfortably contains the
    #      empirically-observed congested band (Y[45.5, 48.0]) with margin
    #      on both sides, so it can't be dodged by hugging either pad row.
    #   X: the camped extent measured directly from the crossing inventory
    #      (F.Cu occupied X[42.7, 124.5], B.Cu X[43.7 or less, 124.3]),
    #      rounded outward to X[42.0, 126.0] so the region fully covers
    #      both layers' occupied span with a little slack; harmless to be
    #      generous since this is a cost, not a block.
    #
    # Penalty magnitude arithmetic (grid pitch 0.1mm, so 1 cell = 0.1mm;
    # base LATERAL step cost = 1.0/cell orthogonal, DIAG=1.414/cell; via
    # cost when not free = 12.0 flat; TURN_PENALTY = 0.3 additive):
    #   - A perpendicular crossing of the region is ~3.4mm = 34 cells (the
    #     region's own Y-height). At per_cell_penalty=4.0 (total cost per
    #     cell ~= 1.0+4.0 = 5.0), that's ~34*5.0 = 170 units.
    #   - A 30mm+ detour around the region (the only alternative before
    #     this fix, per the crossing inventory) costs >=~300 cells * ~1.0-
    #     1.3/cell (allowing for some turns) = ~300-390 units. 170 << 300,
    #     so a direct crossing stays decisively cheaper than detouring --
    #     satisfies "crossings stay cheap."
    #   - A 60-76mm longitudinal run INSIDE the region (what the 5 fan
    #     nets currently do) is ~600-760 cells. At the same penalty:
    #     600-760 * 5.0 = 3000-3800 units. The open south alternative
    #     (Y[74,84], no penalty) covers a comparable J1-to-U1 distance at
    #     ~600-760 cells * ~1.0-1.3/cell (turns) = ~600-1000 units. The
    #     penalized route is ~3.5-4.5x more expensive -- satisfies
    #     "decisively more expensive than the south-strip alternative."
    #   per_cell_penalty=4.0 sits in the middle of the coordinator-
    #   specified 3-5x-base-cost range and clears both bars with margin
    #   on either side (170 vs 300-390; 3000-3800 vs 600-1000) rather than
    #   sitting right at a threshold.
    (42.0, 44.925, 126.0, 48.320, 4.0),

    # Region 2 (2026-08-11): the J3-pin-2 "pocket mouth" -- see docs/
    # layout-plan.md and the delivery report for the DRC finding this
    # fixes. /MCU_TX (F.Cu) and /MC_XQB2 (B.Cu) both currently ride a
    # shared Y=83.6 corridor (live query, both nets independently)
    # directly above J3's pin row, leaving the GND zone only a 0.5mm gap
    # to pour through to reach J3 pin 2 (/GND): pad copper top edge Y =
    # 84.225 (live query) minus the tracks' own south edge at
    # 83.6 + SIG_WIDTH/2 (0.125) = 83.725 -> 0.5mm.
    #
    # Required channel per the coordinator's formula (zone clearance +
    # zone min width + clearance, all read from the live GND zone
    # settings -- GND_pour_F.Cu: clearance=0.150, minThickness=0.150):
    # 0.15 + 0.15 + 0.15 = 0.45mm. The existing 0.5mm gap clears that by
    # only 0.05mm -- real margin, not zero, but too thin for the zone
    # filler to reliably render a connected sliver there in practice
    # (confirmed: it doesn't, that's the whole DRC finding).
    #
    # Fix: tax the current corridor's own Y-band so the router prefers
    # shifting both nets north instead of detouring around (a straight
    # perpendicular reroute, not a longer path -- there's nothing east/
    # west of this corridor for them to route around anyway, per the
    # earlier BFS finding that the pocket has no side openings either).
    # X[50.0, 80.0]: covers both nets' current horizontal run (/MCU_TX
    # measured X[58.1, 77.9], /MC_XQB2 X[52.0, 78.8], live query),
    # rounded outward for slack, same as region 1's X derivation.
    # Y[82.35, 84.225]: south edge = J3's pin-row pad top edge exactly
    # (84.225) -- GND is poured, not track-routed, so nothing here taxes
    # J3.2's own connection, only the two crossing nets. North edge is
    # 84.225 - 1.875 = 82.35, i.e. taxing 1.875mm north of the pad edge --
    # enough that avoiding the whole band entirely (the cheapest strategy
    # for a ~28mm-long horizontal run, since skimming the edge still costs
    # nearly as much as the full detour) puts the new corridor's south
    # edge at >=82.35, giving a final channel of >=84.225-82.35=1.875mm --
    # 4.2x the 0.45mm requirement, not a bare pass.
    #
    # Penalty: same 4.0 as region 1 -- same reasoning applies (a single
    # perpendicular pin-approach stub, the only thing that still needs to
    # enter this Y-band briefly to reach J3.3/J3.4's own pins, stays cheap
    # relative to any alternative; a full ~28mm horizontal run through it
    # does not).
    (50.0, 82.35, 80.0, 84.225, 4.0),
]

_SOFT_COST_GRID = None


def build_soft_cost_grid():
    grid = [0.0] * NUM_CELLS
    for (rx0, ry0, rx1, ry1, penalty) in SOFT_COST_REGIONS:
        c0 = math.floor((rx0 - X0) / PITCH)
        c1 = math.ceil((rx1 - X0) / PITCH)
        r0 = math.floor((ry0 - Y0) / PITCH)
        r1 = math.ceil((ry1 - Y0) / PITCH)
        r0, r1 = clip_range(r0, r1, ROWS)
        c0, c1 = clip_range(c0, c1, COLS)
        for layer in (0, 1):
            base = layer * CELLS_PER_LAYER
            for row in range(r0, r1 + 1):
                off = base + row * COLS
                for col in range(c0, c1 + 1):
                    idx = off + col
                    if penalty > grid[idx]:
                        grid[idx] = penalty
    return grid


def get_soft_cost_grid():
    global _SOFT_COST_GRID
    if _SOFT_COST_GRID is None:
        _SOFT_COST_GRID = build_soft_cost_grid()
    return _SOFT_COST_GRID


# --- per-net obstacle grid -------------------------------------------------

def build_blocked_for_net(static_blocked, all_pads, active_nets, current_net, half_width):
    blocked = bytearray(static_blocked)
    margin = CLEARANCE + half_width
    for p in all_pads:
        if p["net"] == current_net:
            continue
        x0, y0, x1, y1 = p["bbox"]
        rasterize_rect_mm(blocked, x0 - margin, y0 - margin, x1 + margin, y1 + margin, p["layers"])
    for name, data in active_nets.items():
        if name == current_net:
            continue
        w = net_width(name)
        seg_margin = CLEARANCE + half_width + w / 2.0
        for (layer, x0, y0, x1, y1) in data["segments"]:
            rasterize_segment_mm(blocked, layer, x0, y0, x1, y1, seg_margin)
        via_margin = CLEARANCE + half_width + VIA_PAD / 2.0
        for (vx, vy) in data["vias"]:
            rasterize_rect_mm(blocked, vx - via_margin, vy - via_margin, vx + via_margin, vy + via_margin, (0, 1))
    # PRE_ROUTES: locked from before phase 1 even starts, independent of
    # active_nets/locked_active timing entirely -- see PRE_ROUTES' module
    # comment for why this can't just be another active_nets entry (the
    # owning net's own escape-stub edge would see it too early and short-
    # circuit before ever reaching J1/J2).
    for pr in _PRE_ROUTE_SEGMENTS:
        if pr["net"] == current_net:
            continue
        seg_margin = CLEARANCE + half_width + pr["width"] / 2.0
        rasterize_segment_mm(blocked, pr["layer"], pr["x"], pr["y0"], pr["x"], pr["y1"], seg_margin)
    return blocked


# --- A* --------------------------------------------------------------------

LATERAL = [
    (-1, -1, DIAG), (-1, 0, 1.0), (-1, 1, DIAG),
    (0, -1, 1.0), (0, 1, 1.0),
    (1, -1, DIAG), (1, 0, 1.0), (1, 1, DIAG),
]


def octile_h(row, col, targets):
    best = None
    for (trow, tcol) in targets:
        dr = abs(row - trow)
        dc = abs(col - tcol)
        h = (dr + dc) - (2 - DIAG) * min(dr, dc)
        if best is None or h < best:
            best = h
    return best if best is not None else 0.0


def astar(blocked, free_cross_rc, start_nodes, goal_cells, attractor_rc, soft_cost=None):
    """start_nodes: list of (layer,row,col). goal_cells: set of flat nids that
    count as already-reached (the growing net tree). attractor_rc: list of
    (row,col) used only for the heuristic. soft_cost: optional list/array,
    NUM_CELLS long, of an ADDITIVE per-cell cost charged on top of the base
    step/via cost for expanding INTO that cell (see SOFT_COST_REGIONS) --
    unlike `blocked`, this never prevents a move, it only prices it, same
    mechanism as TURN_PENALTY. Returns list of (layer,row,col) path nodes
    from a start node to the goal, or None.

    Heap entries are (f_cost, counter, node_id): counter is a strictly
    monotonic push sequence number, so no two heap entries ever compare
    equal on the tiebreaker -- equal-f_cost frontier expansion order is
    already fully determined by push order, which is itself deterministic
    (LATERAL is a fixed list, start_nodes/attractor_rc come from
    already-sorted pad data). This was verified, not changed."""
    dist = {}
    came_from = {}
    arrival_dir = {}
    open_heap = []
    counter = 0
    for (layer, row, col) in start_nodes:
        n = nid(layer, row, col)
        if blocked[n]:
            continue
        dist[n] = 0.0
        arrival_dir[n] = None
        h = octile_h(row, col, attractor_rc)
        heapq.heappush(open_heap, (h, counter, n))
        counter += 1
    visited = set()
    expansions = 0
    heartbeat_at = 200_000
    while open_heap:
        f, _, cur = heapq.heappop(open_heap)
        if cur in visited:
            continue
        visited.add(cur)
        expansions += 1
        if expansions >= heartbeat_at:
            log(f"    astar heartbeat: {expansions} nodes expanded, heap={len(open_heap)}, f={f:.1f}")
            heartbeat_at += 200_000
        if cur in goal_cells:
            path = [cur]
            while path[-1] in came_from:
                path.append(came_from[path[-1]])
            path.reverse()
            out = []
            for n in path:
                layer, rem = divmod(n, CELLS_PER_LAYER)
                row, col = divmod(rem, COLS)
                out.append((layer, row, col))
            return out
        g = dist[cur]
        cur_dir = arrival_dir[cur]
        layer, rem = divmod(cur, CELLS_PER_LAYER)
        row, col = divmod(rem, COLS)

        for dr, dc, step_cost in LATERAL:
            nrow, ncol = row + dr, col + dc
            if not (0 <= nrow < ROWS and 0 <= ncol < COLS):
                continue
            nn = nid(layer, nrow, ncol)
            if blocked[nn] or nn in visited:
                continue
            new_dir = (dr, dc)
            turn = 0.0 if (cur_dir is None or cur_dir == new_dir) else TURN_PENALTY
            penalty = soft_cost[nn] if soft_cost is not None else 0.0
            new_g = g + step_cost + turn + penalty
            if new_g < dist.get(nn, math.inf):
                dist[nn] = new_g
                came_from[nn] = cur
                arrival_dir[nn] = new_dir
                h = octile_h(nrow, ncol, attractor_rc)
                heapq.heappush(open_heap, (new_g + h, counter, nn))
                counter += 1

        other_layer = 1 - layer
        nn = nid(other_layer, row, col)
        if not blocked[nn] and nn not in visited:
            via_cost = 0.0 if (row, col) in free_cross_rc else VIA_COST
            penalty = soft_cost[nn] if soft_cost is not None else 0.0
            new_g = g + via_cost + penalty
            if new_g < dist.get(nn, math.inf):
                dist[nn] = new_g
                came_from[nn] = cur
                arrival_dir[nn] = cur_dir
                h = octile_h(row, col, attractor_rc)
                heapq.heappush(open_heap, (new_g + h, counter, nn))
                counter += 1
    return None


def path_length_mm(path_rc):
    total = 0.0
    for (l0, r0, c0), (l1, r1, c1) in zip(path_rc, path_rc[1:]):
        if l0 != l1:
            continue
        dx = (c1 - c0) * PITCH
        dy = (r1 - r0) * PITCH
        total += math.hypot(dx, dy)
    return total


def path_to_geometry(path_rc, free_cross_rc):
    n = len(path_rc)
    corners = [path_rc[0]]
    for k in range(1, n - 1):
        pl, pr, pc = path_rc[k - 1]
        cl, cr, cc = path_rc[k]
        nl, nr, nc = path_rc[k + 1]
        if cl != pl or cl != nl:
            corners.append(path_rc[k])
            continue
        d1 = (cr - pr, cc - pc)
        d2 = (nr - cr, nc - cc)
        if d1 != d2:
            corners.append(path_rc[k])
    corners.append(path_rc[-1])

    segments = []
    vias = []
    for i in range(len(corners) - 1):
        l0, r0, c0 = corners[i]
        l1, r1, c1 = corners[i + 1]
        x0, y0 = to_mm(r0, c0)
        x1, y1 = to_mm(r1, c1)
        if l0 != l1:
            if (r0, c0) not in free_cross_rc:
                vias.append((x0, y0))
            continue
        if (r0, c0) == (r1, c1):
            continue
        segments.append((l0, x0, y0, x1, y1))
    return segments, vias


# --- MST ---------------------------------------------------------------

def prim_mst(pad_pts):
    """pad_pts must already be in a deterministic order -- group_nets()
    guarantees (ref, pin) sorted order, so pad_pts[0] (the MST root) and
    every equal-distance tie inside the nested loop below resolve the same
    way on every run."""
    n = len(pad_pts)
    in_tree = [False] * n
    in_tree[0] = True
    order = [0]
    edges = []
    for _ in range(n - 1):
        best = None
        for i in range(n):
            if not in_tree[i]:
                continue
            for j in range(n):
                if in_tree[j]:
                    continue
                d = math.hypot(pad_pts[i]["x"] - pad_pts[j]["x"], pad_pts[i]["y"] - pad_pts[j]["y"])
                if best is None or d < best[2]:
                    best = (i, j, d)
        i, j, d = best
        in_tree[j] = True
        order.append(j)
        edges.append((i, j, d))
    return order, edges


# --- board writers ----------------------------------------------------

def add_track(board, net, layer_id, x0, y0, x1, y1, width, uuid_key):
    seg = pcbnew.PCB_TRACK(board)
    seg.SetStart(pcbnew.VECTOR2I(fmm(x0), fmm(y0)))
    seg.SetEnd(pcbnew.VECTOR2I(fmm(x1), fmm(y1)))
    seg.SetWidth(fmm(width))
    seg.SetLayer(layer_id)
    seg.SetNet(net)
    board.Add(seg)
    set_uuid(seg, uuid_key)
    return seg


def add_via(board, net, x, y, uuid_key):
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(pcbnew.VECTOR2I(fmm(x), fmm(y)))
    via.SetDrill(fmm(VIA_DRILL))
    via.SetWidth(fmm(VIA_PAD))
    via.SetViaType(pcbnew.VIATYPE_THROUGH)
    via.SetNetCode(net.GetNetCode())
    board.Add(via)
    set_uuid(via, uuid_key)
    return via


def clear_all_tracks(board, keep_alive):
    removed_tracks = 0
    removed_vias = 0
    for t in list(board.Tracks()):
        if t.Type() == pcbnew.PCB_VIA_T:
            removed_vias += 1
        else:
            removed_tracks += 1
        board.Remove(t)
        keep_alive.append(t)
    return removed_tracks, removed_vias


# --- net-level routing --------------------------------------------------
#
# route_edges() routes a SLICE [edge_lo:edge_hi) of a net's precomputed
# Prim's-MST edge list, optionally continuing from a `seed` state (the
# tree_cells/attractors/segments/vias left behind by an earlier slice).
# This lets main() run routing in two global passes -- see its "PHASE 1 /
# PHASE 2" comment for why:
#   phase 1: edges[0:1] for every net (the escape stub off J1/J2), seed=None
#   phase 2: edges[1:]  for every net, seed=phase 1's result
# route_net() is the old single-call, single-phase entry point, now a thin
# wrapper -- add_ground_pour.py and callers that don't care about phasing
# can still call it directly.

def route_edges(net_name, pads, all_pads, active_nets, free_cross_rc,
                 order, edges, edge_lo, edge_hi, seed=None):
    """Route edges[edge_lo:edge_hi] of this net's MST. Returns dict with
    success, segments, vias, width, edge_results, tree_cells, attractors
    (the last two are state a later phase can pass back in as `seed`), or
    success=False."""
    half_width = max(net_width(net_name) / 2.0, VIA_PAD / 2.0)
    static_blocked = get_static_blocked(half_width)
    blocked = build_blocked_for_net(static_blocked, all_pads, active_nets, net_name, half_width)

    if seed is None:
        tree_cells = set()
        root = pads[order[0]]
        root_rc = to_grid(root["x"], root["y"])
        for layer in root["layers"]:
            tree_cells.add(nid(layer, root_rc[0], root_rc[1]))
        attractors = [root_rc]
        all_segments = []
        all_vias = []
        edge_results = []
    else:
        tree_cells = set(seed["tree_cells"])
        attractors = list(seed["attractors"])
        all_segments = list(seed["segments"])
        all_vias = list(seed["vias"])
        edge_results = list(seed["edge_results"])

    for edge_idx in range(edge_lo, edge_hi):
        i, j, dist_mm = edges[edge_idx]
        new_pad = pads[j]
        new_rc = to_grid(new_pad["x"], new_pad["y"])

        # Multi-target start: also accept a path that starts from any pad
        # this edge's own target (j) is DIRECTLY MST-adjacent to via a
        # LATER edge in this net's full edge list (edge_idx' > edge_idx),
        # NOT bounded by this call's edge_hi -- see PHASE1_LONGEST_EDGE_NETS'
        # comment for why: once a net's long edge moves into phase 1 while
        # its sibling short edge stays in phase 2, the two are in different
        # route_edges() calls entirely, so bounding this lookahead at
        # edge_hi would blind phase 1's long edge to the exact sibling
        # multi-target exists to help it use. Safe regardless of which
        # phase the later edge falls in: that edge still exists and still
        # explicitly connects the two pads to each other regardless of
        # which one this edge ends up using -- if the alternate wins here,
        # its cells are already in tree_cells (carried across the phase1-
        # >phase2 seed) by the time that later edge runs, so its own astar
        # (start=itself, goal=tree_cells) trivially succeeds (0-length,
        # already connected); if new_pad (j) wins as before, the later
        # edge behaves exactly as it always did. Either way every pad
        # still ends up tied into the same tree -- this only changes WHICH
        # physical pad of an MST-adjacent pair claims a contested approach
        # first, it never removes or reorders an edge. Lets an edge
        # complete via a same-net sibling pad whose own approach is clear
        # even when the MST-nominal pad's is walled off (see
        # PRIORITY_NETS' comment for the /+3V3 case this fixes).
        # EXCEPT a pad that's a PRE_ROUTES endpoint for this net: that pad's
        # connection is handled entirely by splice_pre_route_into_result()
        # (deterministic, not searched, per PRE_ROUTES' whole design) --
        # offering it here would let this general mechanism "help" by
        # re-discovering a real A* path to it, duplicating the pre-routed
        # track with genuine (redundant, wasteful, and un-audited) copper.
        # Found in a live run: /TH2's escape-stub edge (J1.1->U1.26) pulled
        # in R1.2 as an alt via this exact lookahead once it was widened to
        # the full edge list for PHASE1_LONGEST_EDGE_NETS, and the orphan
        # follow-up below then drew a second, real 5.8mm track to R1.2 on
        # top of the pre-route -- not incorrect (same net, not a short) but
        # not what "not searched" was supposed to mean either.
        pre_routed_pads = {pr["to"] for pr in _PRE_ROUTE_SEGMENTS if pr["net"] == net_name}
        pre_routed_pads |= {pr["from"] for pr in _PRE_ROUTE_SEGMENTS if pr["net"] == net_name}
        alt_pads = []
        for e2 in range(edge_idx + 1, len(edges)):
            ei2, ej2, _ = edges[e2]
            if ei2 == j:
                cand = pads[ej2]
            elif ej2 == j:
                cand = pads[ei2]
            else:
                continue
            if (cand["ref"], cand["pin"]) in pre_routed_pads:
                continue
            alt_pads.append(cand)
        start_pads = [new_pad] + alt_pads
        start_nodes = []
        for p in start_pads:
            rc = to_grid(p["x"], p["y"])
            start_nodes += [(layer, rc[0], rc[1]) for layer in p["layers"]]

        edge_t0 = time.time()
        alt_note = "" if not alt_pads else (
            f" (+ alt target(s) {[(p['ref'], p['pin']) for p in alt_pads]})")
        log(f"  edge {edge_idx+1}/{len(edges)} {net_name}: {pads[i]['ref']}.{pads[i]['pin']} -> "
            f"{pads[j]['ref']}.{pads[j]['pin']}{alt_note} (mst_dist={dist_mm:.2f}mm) astar start")
        path = astar(blocked, free_cross_rc, start_nodes, tree_cells, attractors,
                     soft_cost=get_soft_cost_grid())
        edge_dt = time.time() - edge_t0
        if path is None:
            log(f"  edge {edge_idx+1}/{len(edges)} {net_name}: NO PATH after {edge_dt:.2f}s")
            return {"success": False}

        # Which pad the path actually started from -- may be an alt_pad,
        # not the MST-nominal new_pad -- purely for accurate logging/
        # edge_results; path correctness doesn't depend on this.
        used_pad = new_pad
        used_rc = new_rc
        start_layer, start_row, start_col = path[0]
        for p in start_pads:
            rc = to_grid(p["x"], p["y"])
            if (start_row, start_col) == rc and start_layer in p["layers"]:
                used_pad = p
                used_rc = rc
                break

        log(f"  edge {edge_idx+1}/{len(edges)} {net_name}: routed in {edge_dt:.2f}s, "
            f"path_len={path_length_mm(path):.2f}mm"
            + (f", via {used_pad['ref']}.{used_pad['pin']}" if used_pad is not new_pad else ""))
        segs, vias = path_to_geometry(path, free_cross_rc)
        all_segments.extend(segs)
        all_vias.extend(vias)
        for (layer, row, col) in path:
            tree_cells.add(nid(layer, row, col))
        attractors.append(used_rc)
        edge_results.append({
            "from": (pads[i]["ref"], pads[i]["pin"]),
            "to": (used_pad["ref"], used_pad["pin"]),
            "length_mm": path_length_mm(path),
        })

        # BUG FIXED 2026-08-11 (found via DRC on the pre-route run: U1.1/
        # U1.2 both /+3V3, showed up as "missing connection" even though
        # route_board.py reported the net fully routed): if the winning
        # start was an alt_pad instead of this edge's own MST-nominal
        # new_pad, new_pad itself is now silently ORPHANED -- the
        # precomputed MST edge list is fixed at prim_mst() time and has no
        # idea a runtime substitution happened, so whichever LATER edge
        # was originally going to connect new_pad no longer will (it now
        # finds new_pad... no, it finds ITS OWN nominal target already
        # satisfied and degenerates to a no-op instead, leaving new_pad
        # with no edge responsible for it at all). Explicitly connect
        # every OTHER candidate in start_pads too, not just the winner --
        # any that are already in tree_cells (the common case: the pad
        # this same alt-swap was borrowed FROM) degenerate to the same
        # free 0-length no-op the multi-target design always relied on;
        # any that genuinely aren't yet connected get a real edge here
        # instead of being dropped.
        # Goal is used_pad specifically (not the whole tree_cells): these
        # candidates are MST-siblings by construction (that's the only way
        # they ended up in start_pads together -- see alt_pads' lookahead
        # above), meant to connect to EACH OTHER with a short direct hop,
        # same as the "later edge" this substitutes for would have done.
        # Targeting the full tree risks astar wandering off toward whatever
        # part of the tree looks cheapest by the heuristic instead of the
        # short sibling hop, which needlessly inflates both this search's
        # cost and the resulting copper another net has to route around.
        used_goal_cells = {nid(layer, used_rc[0], used_rc[1]) for layer in used_pad["layers"]}
        for other in start_pads:
            if other is used_pad:
                continue
            other_rc = to_grid(other["x"], other["y"])
            other_start = [(layer, other_rc[0], other_rc[1]) for layer in other["layers"]]
            other_t0 = time.time()
            other_path = astar(blocked, free_cross_rc, other_start, used_goal_cells, [used_rc],
                                soft_cost=get_soft_cost_grid())
            other_dt = time.time() - other_t0
            if other_path is None:
                log(f"  edge {edge_idx+1}/{len(edges)} {net_name}: follow-up connect for "
                    f"{other['ref']}.{other['pin']} (orphaned by the {used_pad['ref']}."
                    f"{used_pad['pin']} alt-swap above) NO PATH after {other_dt:.2f}s")
                return {"success": False}
            other_segs, other_vias = path_to_geometry(other_path, free_cross_rc)
            if other_segs or other_vias:
                log(f"  edge {edge_idx+1}/{len(edges)} {net_name}: follow-up connected "
                    f"{other['ref']}.{other['pin']} in {other_dt:.2f}s, "
                    f"path_len={path_length_mm(other_path):.2f}mm")
            all_segments.extend(other_segs)
            all_vias.extend(other_vias)
            for (layer, row, col) in other_path:
                tree_cells.add(nid(layer, row, col))

    width = net_width(net_name)
    return {
        "success": True,
        "segments": [(l, x0, y0, x1, y1) for (l, x0, y0, x1, y1) in all_segments],
        "vias": all_vias,
        "width": width,
        "edge_results": edge_results,
        "tree_cells": tree_cells,
        "attractors": attractors,
    }


def route_net(net_name, pads, all_pads, active_nets, free_cross_rc):
    """Single-phase convenience wrapper: route a net's full MST in one call."""
    order, edges = prim_mst(pads)
    return route_edges(net_name, pads, all_pads, active_nets, free_cross_rc,
                        order, edges, 0, len(edges))


def bbox_of_net_data(data, pad_bbox=None, margin=0.0):
    xs = []
    ys = []
    if pad_bbox:
        xs += [pad_bbox[0], pad_bbox[2]]
        ys += [pad_bbox[1], pad_bbox[3]]
    for (layer, x0, y0, x1, y1) in data.get("segments", []):
        xs += [x0, x1]
        ys += [y0, y1]
    for (vx, vy) in data.get("vias", []):
        xs += [vx, vx]
        ys += [vy, vy]
    if not xs:
        return None
    return (min(xs) - margin, min(ys) - margin, max(xs) + margin, max(ys) + margin)


def rects_overlap(a, b):
    return not (a[2] < b[0] or b[2] < a[0] or a[3] < b[1] or b[3] < a[1])


# --- rip-up/retry driver, run once per phase -------------------------

def ripup_route_phase(phase_label, net_order, nets, all_pads, free_cross_rc,
                       mst_cache, edge_range_fn, seed_fn, iteration_cap, ripup_margin_mm,
                       locked_active=None):
    """Rip-up/retry queue driver for one routing phase. edge_range_fn(net_name)
    -> (edge_lo, edge_hi): which slice of that net's precomputed MST edges to
    route this phase (nets with edge_lo >= edge_hi have nothing to do this
    phase and trivially pass their seed state through unchanged).
    seed_fn(net_name) -> seed dict for route_edges(), or None to start fresh
    from the MST root.

    locked_active, if given, is {net_name: {"segments":..., "vias":...}} --
    geometry from a PRIOR phase that's permanently fixed: every other net's
    obstacle grid always includes it, but rip-up can only ever revert a net
    back to this baseline, never delete it outright. This is the mechanism
    that fixes the original livelock (see RIPUP_MARGIN_MM's comment and this
    function's caller in main()): a net's escape stub, once locked in during
    phase 1, can never again be the thing a later rip-up cascade deletes, so
    phase 2's trunk routing can only ever cycle its OWN trunk attempts, not
    unravel and re-fight over the connector escape lanes.

    Returns (results, status, cap_hit): results maps net_name -> the route_edges()
    success dict (only for nets that completed edge_range_fn's slice, merged
    with any locked_active/seed geometry); status maps net_name -> 'first_try'
    | 'ripup' | 'failed' (only for nets actually attempted -- nets skipped
    entirely, i.e. edge_lo>=edge_hi, are not present)."""
    active_nets = dict(locked_active) if locked_active else {}
    results = {}
    status = {}
    attempt_count = {n: 0 for n in net_order}
    tier2_used = set()
    queue = deque(net_order)
    iterations = 0
    cap_hit = False

    while queue:
        iterations += 1
        if iterations > iteration_cap:
            cap_hit = True
            log(f"[{phase_label}] WARNING: iteration cap ({iteration_cap}) hit; "
                f"remaining queue: {list(queue)}")
            break

        net_name = queue.popleft()
        edge_lo, edge_hi = edge_range_fn(net_name)
        order, edges = mst_cache[net_name]

        if edge_lo >= edge_hi:
            # nothing to route this phase (e.g. phase 2 for a 2-pad net
            # phase 1 already finished) -- pass the seed through unchanged
            seed = seed_fn(net_name)
            passthrough = {
                "success": True,
                "segments": list(seed["segments"]),
                "vias": list(seed["vias"]),
                "width": net_width(net_name),
                "edge_results": list(seed["edge_results"]),
                "tree_cells": set(seed["tree_cells"]),
                "attractors": list(seed["attractors"]),
            }
            results[net_name] = passthrough
            active_nets[net_name] = passthrough
            status[net_name] = "first_try"
            continue

        attempt_count[net_name] += 1
        seed = seed_fn(net_name)
        t0 = time.time()
        log(f"[{phase_label}] iter {iterations}/{iteration_cap} net={net_name} "
            f"attempt#{attempt_count[net_name]} queue_len={len(queue)} "
            f"active_nets={len(active_nets)} tier2={net_name in tier2_used}")
        result = route_edges(net_name, nets[net_name], all_pads, active_nets,
                              free_cross_rc, order, edges, edge_lo, edge_hi, seed=seed)
        dt = time.time() - t0
        log(f"[{phase_label}] iter {iterations} net={net_name} "
            f"{'SUCCESS' if result['success'] else 'FAIL'} in {dt:.2f}s")

        if result["success"]:
            active_nets[net_name] = result
            results[net_name] = result
            status.setdefault(net_name, "first_try" if attempt_count[net_name] == 1 else "ripup")
            continue

        # failed -- try rip-up
        status[net_name] = "ripup"
        pad_xs = [p["x"] for p in nets[net_name]]
        pad_ys = [p["y"] for p in nets[net_name]]
        fail_bbox = (min(pad_xs) - ripup_margin_mm, min(pad_ys) - ripup_margin_mm,
                     max(pad_xs) + ripup_margin_mm, max(pad_ys) + ripup_margin_mm)

        # Never-attempted-this-phase locked nets are excluded from
        # "rippable": their active_nets entry already equals their locked
        # baseline, so "ripping them up" would be a same-state no-op, not
        # progress.
        untouched_locked = (set(locked_active) - set(results)) if locked_active else set()
        rippable = set(active_nets) - {net_name} - untouched_locked

        if net_name not in tier2_used:
            blockers = sorted(
                name for name in rippable
                if (bb := bbox_of_net_data(active_nets[name])) and rects_overlap(bb, fail_bbox)
            )
        else:
            blockers = sorted(rippable)

        if not blockers:
            if net_name in tier2_used:
                status[net_name] = "failed"
                print(f"[{phase_label}] {net_name}: FAILED -- no path even with every "
                      f"rippable net ripped up")
                continue
            tier2_used.add(net_name)
            queue.append(net_name)
            continue

        log(f"[{phase_label}] iter {iterations} net={net_name}: ripping up "
            f"{len(blockers)} blocker(s): {blockers}")
        for b in blockers:
            if locked_active and b in locked_active:
                active_nets[b] = locked_active[b]  # revert to this phase's fixed baseline
            else:
                del active_nets[b]
            results.pop(b, None)
            queue.append(b)
        queue.append(net_name)

    return results, status, cap_hit


# --- main -----------------------------------------------------------------

def main():
    global _MOUNTING_HOLES, _PRE_ROUTE_SEGMENTS

    board = pcbnew.LoadBoard(PCB_PATH)
    _kept_alive = []

    removed_tracks, removed_vias = clear_all_tracks(board, _kept_alive)
    print(f"[route] cleared {removed_tracks} existing track segment(s), {removed_vias} existing via(s)")

    mounting_holes = []
    for ref in MOUNTING_HOLE_REFS:
        for fp in board.GetFootprints():
            if fp.GetReference() == ref:
                pad = fp.Pads()[0]
                pos = pad.GetPosition()
                drill = pad.GetDrillSize()
                mounting_holes.append((mm(pos.x), mm(pos.y), mm(drill.x)))
                break
    print(f"[route] mounting holes: {[(r, round(x,3), round(y,3), round(d,3)) for r,(x,y,d) in zip(MOUNTING_HOLE_REFS, mounting_holes)]}")
    _MOUNTING_HOLES = mounting_holes  # read by get_static_blocked() -- see its docstring

    all_pads = collect_pads(board)
    nets = group_nets(all_pads)
    print(f"[route] grid {COLS}x{ROWS} cells/layer, pitch {PITCH}mm, {len(nets)} routable net(s)")

    # free layer-crossing cells: every through-hole pad's grid cell (both
    # layers connect there for free -- it's a plated hole, not a via)
    free_cross_rc = set()
    for p in all_pads:
        if p["is_th"]:
            free_cross_rc.add(to_grid(p["x"], p["y"]))

    # PRE_ROUTES: resolved and locked BEFORE phase 1 starts, so every net's
    # phase-1 stub routing (not just phase 2) already treats them as fixed
    # obstacles -- see PRE_ROUTES' module comment.
    _PRE_ROUTE_SEGMENTS = resolve_pre_routes(all_pads)

    # --- net ordering ---
    mst_cache = {n: prim_mst(nets[n]) for n in nets}

    def mst_length(name):
        _, edges = mst_cache[name]
        return sum(e[2] for e in edges)

    # Sort key is (mst_length, net_name): name is the final, total
    # tie-break, so nets with equal (or float-equal) MST length never fall
    # back on dict/list insertion order to decide who routes first.
    #
    # priority goes first, in PRIORITY_NETS' own fixed order (not
    # mst_length-sorted -- it's a short, explicitly-authored list, so its
    # own order is already the deterministic, documented ordering rule).
    priority = [n for n in PRIORITY_NETS if n in nets]
    safety = [n for n in ("/MC_HOT1", "/MC_HOT2") if n in nets and n not in priority]
    power = [n for n in ("/+5V", "/+3V3") if n in nets and n not in priority]
    rest = [n for n in nets if n not in priority and n not in safety and n not in power]
    safety.sort(key=lambda n: (mst_length(n), n))
    power.sort(key=lambda n: (mst_length(n), n))
    rest.sort(key=lambda n: (mst_length(n), n))
    net_order = priority + safety + power + rest
    print(f"[route] net order: {net_order}")

    # --- PHASE 1: escape-stub pass -----------------------------------
    # Route ONLY each net's first MST edge (the root pad -- always a J1 or
    # J2 connector pin for this board's nets -- to its nearest neighbour,
    # in practice almost always that pin's TP) for every net, before ANY
    # net's remaining trunk edges exist on the board.
    #
    # Root cause this works around (found via the per-net/per-edge timed
    # logging above): J1 (14 pins) and J2 (15 pins) are SMD, B.Cu-only,
    # 1.25mm pitch -- a genuinely tight escape corridor. Routing nets whole
    # (stub+trunk together, in mst_length order, as the previous version
    # did) interleaves one net's TRUNK edges -- which loop back out through
    # the same connector's pin row on their way to U1 -- with a SIBLING
    # pin's STILL-UNROUTED stub. Whichever sibling's stub is attempted
    # last ends up walled in by its neighbours' trunk traces, fails fast
    # (its local pocket is small, so A* exhausts it in well under a
    # second), and the rip-up/retry logic then tears up ~half the board
    # trying to make room -- but the exact same sibling gets walled in
    # again next cycle, forever (confirmed: active_nets oscillated between
    # 0 and ~20/29 for 600+ seconds without ever converging; verified
    # separately that both permanently-stuck nets route in <0.2s each when
    # given the whole empty board to themselves). Routing every stub before
    # any trunk exists removes the walling-in mechanism entirely: all 29
    # pins claim their own short slice of escape space while the corridor
    # is still empty, in roughly the order they're placed along the row.
    log("=== PHASE 1: escape-stub pass (every net's root->nearest-pad edge, "
        "board otherwise empty) ===")
    phase1_results, phase1_status, phase1_cap_hit = ripup_route_phase(
        "phase1", net_order, nets, all_pads, free_cross_rc, mst_cache,
        edge_range_fn=lambda n: (0, phase1_edge_end(n, mst_cache)),
        seed_fn=lambda n: None,
        iteration_cap=RIP_UP_ITERATION_CAP,
        ripup_margin_mm=RIPUP_MARGIN_MM,
        locked_active=None,
    )
    phase1_failed = [n for n in net_order if n not in phase1_results]
    if phase1_failed:
        print(f"[route] PHASE 1 FAILED -- these nets have no escape route at all, "
              f"even with the whole board otherwise empty: {phase1_failed}")

    # Splice PRE_ROUTES into their owning nets' phase-1 results now that
    # each net's own escape-stub edge has genuinely connected it to J1/J2
    # -- see splice_pre_route_into_result()'s docstring for why this must
    # happen here (after phase 1) and not earlier.
    for pr in _PRE_ROUTE_SEGMENTS:
        if pr["net"] in phase1_results:
            phase1_results[pr["net"]] = splice_pre_route_into_result(phase1_results[pr["net"]], pr)
            log(f"[route] PRE_ROUTE spliced into {pr['net']}'s phase-1 result: "
                f"{pr['from'][0]}.{pr['from'][1]} <-> {pr['to'][0]}.{pr['to'][1]}")
        else:
            print(f"[route] WARNING: PRE_ROUTE for {pr['net']} not spliced -- "
                  f"that net's own phase-1 escape stub never succeeded")

    # --- PHASE 2: trunk pass ------------------------------------------
    # Continue every net that has one from where its stub left off. Each
    # net's phase-1 stub geometry is LOCKED (locked_active): it always
    # blocks every other net's obstacle grid, but rip-up can only ever
    # revert a net back to its stub baseline, never delete it -- so phase 2
    # can only ever re-fight over trunk routing, not reopen the connector
    # escape-lane contention phase 1 already resolved.
    log("=== PHASE 2: trunk pass (continue each net from its locked stub) ===")
    # STUB_UNLOCK_NETS is empty (candidate 2 tried and reverted -- see its
    # comment) so this is equivalent to locking every net's stub, same as
    # before that investigation; kept as a filter rather than removed
    # outright so re-enabling a future narrow exemption is a one-line change.
    locked_active = {n: {"segments": list(r["segments"]), "vias": list(r["vias"])}
                      for n, r in phase1_results.items()
                      if n not in STUB_UNLOCK_NETS}
    phase2_order = [n for n in net_order if n in phase1_results]
    phase2_results, phase2_status, phase2_cap_hit = ripup_route_phase(
        "phase2", phase2_order, nets, all_pads, free_cross_rc, mst_cache,
        edge_range_fn=lambda n: (phase1_edge_end(n, mst_cache), len(mst_cache[n][1])),
        seed_fn=lambda n: phase1_results[n],
        iteration_cap=RIP_UP_ITERATION_CAP,
        ripup_margin_mm=RIPUP_MARGIN_MM,
        locked_active=locked_active,
    )

    active_nets = phase2_results
    net_status = dict(phase1_status)
    net_status.update(phase2_status)
    cap_hit = phase1_cap_hit or phase2_cap_hit

    # --- GND stitching vias ---
    gnd_pads = [p for p in all_pads if p["net"] == "/GND"]
    gnd_net = get_net(board, "/GND")
    single_layer_gnd = [p for p in gnd_pads if len(p["layers"]) == 1]
    gnd_blocked = build_blocked_for_net(get_static_blocked(VIA_PAD / 2.0), all_pads, active_nets, "/GND", VIA_PAD / 2.0)
    gnd_via_count = 0
    for p in single_layer_gnd:
        layer = p["layers"][0]
        row, col = to_grid(p["x"], p["y"])
        found = _bfs_via_drop(gnd_blocked, layer, row, col)
        if found is None:
            print(f"[route] WARNING: could not find a stitching via drop point near GND pad {p['ref']}.{p['pin']}")
            continue
        path_rc = [(layer, r, c) for (r, c) in found]
        if len(path_rc) >= 2:
            corners = [path_rc[0]]
            for k in range(1, len(path_rc) - 1):
                pr, pc = path_rc[k - 1][1], path_rc[k - 1][2]
                cr, cc = path_rc[k][1], path_rc[k][2]
                nr, nc = path_rc[k + 1][1], path_rc[k + 1][2]
                if (cr - pr, cc - pc) != (nr - cr, nc - cc):
                    corners.append(path_rc[k])
            corners.append(path_rc[-1])
            for seg_idx, (a, b) in enumerate(zip(corners, corners[1:])):
                x0, y0 = to_mm(a[1], a[2])
                x1, y1 = to_mm(b[1], b[2])
                add_track(
                    board, gnd_net, LAYER_IDS[layer], x0, y0, x1, y1, SIG_WIDTH,
                    f"sh03:gndstitch:{p['ref']}:{p['pin']}:seg:{seg_idx}",
                )
        vx, vy = to_mm(found[-1][0], found[-1][1])
        add_via(board, gnd_net, vx, vy, f"sh03:gndstitch:{p['ref']}:{p['pin']}:via")
        gnd_via_count += 1
    print(f"[route] GND stitching vias added: {gnd_via_count} (single-layer GND pads: {len(single_layer_gnd)})")

    # --- draw everything ---
    total_vias = 0
    report_rows = []
    for net_name in net_order:
        if net_name not in active_nets:
            report_rows.append((net_name, 0, 0, 0.0, "FAILED"))
            continue
        data = active_nets[net_name]
        net = get_net(board, net_name)
        for seg_idx, (layer, x0, y0, x1, y1) in enumerate(data["segments"]):
            add_track(
                board, net, LAYER_IDS[layer], x0, y0, x1, y1, data["width"],
                f"sh03:route:{net_name}:seg:{seg_idx}",
            )
        for via_idx, (vx, vy) in enumerate(data["vias"]):
            add_via(board, net, vx, vy, f"sh03:route:{net_name}:via:{via_idx}")
        total_vias += len(data["vias"])
        total_len = sum(math.hypot(x1 - x0, y1 - y0) for (_, x0, y0, x1, y1) in data["segments"])
        status = net_status.get(net_name, "first_try")
        report_rows.append((net_name, len(data["segments"]), len(data["vias"]), total_len, status))

    board.Save(PCB_PATH)
    print(f"[route] saved {PCB_PATH}")

    # --- report ---
    print()
    print(f"{'net':22s} {'segs':>5s} {'vias':>5s} {'length_mm':>10s}  status")
    print("-" * 60)
    for (name, nseg, nvia, length, status) in report_rows:
        print(f"{name:22s} {nseg:5d} {nvia:5d} {length:10.3f}  {status}")

    routed_ok = sum(1 for r in report_rows if r[4] != "FAILED")
    failed = [r[0] for r in report_rows if r[4] == "FAILED"]
    print()
    print(f"[route] totals: {routed_ok}/{len(report_rows)} nets routed, {len(failed)} failed, {total_vias + gnd_via_count} total vias "
          f"({gnd_via_count} GND stitching)")
    if failed:
        print(f"[route] FAILED nets: {failed}")
    if cap_hit:
        print("[route] rip-up iteration cap was hit -- see WARNING above")

    # MC_HOT1/MC_HOT2 safety trace lengths
    for net_name, j1_pin, r_ref in (("/MC_HOT1", "4", "R4"), ("/MC_HOT2", "3", "R3")):
        data = active_nets.get(net_name)
        if not data:
            print(f"[route] REPORT {net_name}: FAILED, no length available")
            continue
        match = [e for e in data["edge_results"]
                 if {e["from"], e["to"]} == {("J1", j1_pin), (r_ref, "1")}]
        if match:
            print(f"[route] REPORT {net_name} J1.{j1_pin} -> {r_ref}.1 length: {match[0]['length_mm']:.3f}mm")
        else:
            print(f"[route] REPORT {net_name}: J1.{j1_pin} -> {r_ref}.1 not a direct MST edge; edges: {data['edge_results']}")


def _bfs_via_drop(blocked, pad_layer, start_row, start_col):
    """BFS on pad_layer from (start_row,start_col) for the nearest cell
    that's free on BOTH layers (suitable for a via). Returns the path of
    (row,col) grid cells (excluding the very start cell) or None."""
    other_layer = 1 - pad_layer
    start = (start_row, start_col)
    visited = {start}
    prev = {}
    q = deque([start])
    steps = 0
    while q and steps < 20000:
        steps += 1
        r, c = q.popleft()
        idx_pad = pad_layer * CELLS_PER_LAYER + r * COLS + c
        idx_other = other_layer * CELLS_PER_LAYER + r * COLS + c
        if (r, c) != start and blocked[idx_pad] == 0 and blocked[idx_other] == 0:
            path = [(r, c)]
            while path[-1] in prev:
                path.append(prev[path[-1]])
            path.reverse()
            return path
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                nr, nc = r + dr, c + dc
                if not (0 <= nr < ROWS and 0 <= nc < COLS):
                    continue
                if (nr, nc) in visited:
                    continue
                nidx = pad_layer * CELLS_PER_LAYER + nr * COLS + nc
                if blocked[nidx]:
                    continue
                visited.add((nr, nc))
                prev[(nr, nc)] = (r, c)
                q.append((nr, nc))
    return None


if __name__ == "__main__":
    main()
