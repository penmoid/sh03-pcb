"""report_pads.py - absolute pad geometry report for sh03-controller.kicad_pcb.

Run with KiCad's bundled interpreter:
  python3 scripts/report_pads.py

Read-only: loads the board and prints a plain-text report. Does not modify
or save the board. Output is written to stdout and to docs/pad-report.txt.
"""

import os
import re

import pcbnew

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PCB_PATH = os.path.join(PROJECT_DIR, "sh03-controller.kicad_pcb")
REPORT_PATH = os.path.join(PROJECT_DIR, "docs", "pad-report.txt")


def mm(nm):
    return pcbnew.ToMM(nm)


def pin_sort_key(pin):
    """Numeric-first sort for pin numbers that may contain letters (e.g. 'A1')."""
    m = re.match(r"(\d+)", pin)
    return (int(m.group(1)) if m else 10**9, pin)


def pad_shape_name(pad):
    shape_map = {
        pcbnew.PAD_SHAPE_CIRCLE: "circle",
        pcbnew.PAD_SHAPE_RECTANGLE: "rect",
        pcbnew.PAD_SHAPE_OVAL: "oval",
        pcbnew.PAD_SHAPE_TRAPEZOID: "trapezoid",
        pcbnew.PAD_SHAPE_ROUNDRECT: "roundrect",
        pcbnew.PAD_SHAPE_CHAMFERED_RECT: "chamfered_rect",
        pcbnew.PAD_SHAPE_CUSTOM: "custom",
    }
    return shape_map.get(pad.GetShape(), str(pad.GetShape()))


def pad_layer_str(pad):
    if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH or pad.GetAttribute() == pcbnew.PAD_ATTRIB_NPTH:
        return "*"
    layer_set = pad.GetLayerSet()
    if layer_set.Contains(pcbnew.F_Cu) and not layer_set.Contains(pcbnew.B_Cu):
        return "F.Cu"
    if layer_set.Contains(pcbnew.B_Cu) and not layer_set.Contains(pcbnew.F_Cu):
        return "B.Cu"
    return "*"


def pad_type_str(pad):
    attr = pad.GetAttribute()
    if attr in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH):
        return "TH"
    return "SMD"


def net_name(pad):
    return pad.GetNetname() or "<no net>"


def main():
    board = pcbnew.LoadBoard(PCB_PATH)

    rows = []  # dicts, one per pad
    for fp in board.GetFootprints():
        ref = fp.GetReference()
        for pad in fp.Pads():
            pos = pad.GetPosition()
            size = pad.GetSize()
            is_th = pad_type_str(pad) == "TH"
            drill = pad.GetDrillSize()
            rows.append({
                "ref": ref,
                "pin": pad.GetNumber(),
                "net": net_name(pad),
                "x": mm(pos.x),
                "y": mm(pos.y),
                "w": mm(size.x),
                "h": mm(size.y),
                "shape": pad_shape_name(pad),
                "type": "TH" if is_th else "SMD",
                "layer": pad_layer_str(pad),
                "drill_mm": mm(drill.x) if is_th else None,
            })

    rows.sort(key=lambda r: (r["ref"], pin_sort_key(r["pin"])))

    lines = []

    header = f"{'ref':6s} {'pin':6s} {'net':22s} {'pad_x_mm':>9s} {'pad_y_mm':>9s} {'pad_w_mm':>9s} {'pad_h_mm':>9s} {'shape':10s} {'type':4s} {'layer':6s} {'drill_mm':>9s}"
    lines.append(header)
    lines.append("-" * len(header))
    for r in rows:
        drill_str = f"{r['drill_mm']:.3f}" if r["drill_mm"] is not None else ""
        lines.append(
            f"{r['ref']:6s} {r['pin']:6s} {r['net']:22s} {r['x']:9.3f} {r['y']:9.3f} "
            f"{r['w']:9.3f} {r['h']:9.3f} {r['shape']:10s} {r['type']:4s} {r['layer']:6s} {drill_str:>9s}"
        )

    lines.append("")
    lines.append("=== Summary 1: nets with more than one pad ===")
    # "<no net>" pads (mounting-hole H1-H4, connector mechanical pins) are not
    # a real net -- they aren't electrically joined to each other -- so they're
    # excluded from net grouping and reported separately below.
    real_rows = [r for r in rows if r["net"] != "<no net>"]
    no_net_rows = [r for r in rows if r["net"] == "<no net>"]
    nets = {}
    for r in real_rows:
        nets.setdefault(r["net"], []).append(r)
    multi = {name: pads for name, pads in nets.items() if len(pads) > 1}
    single = sorted(name for name, pads in nets.items() if len(pads) == 1)
    for name in sorted(multi):
        pads = multi[name]
        pad_strs = [f"{p['ref']}.{p['pin']} ({p['x']:.3f}, {p['y']:.3f})" for p in pads]
        lines.append(f"{name}: " + " -> ".join(pad_strs))
    lines.append("")
    lines.append("Single-pad nets: " + ", ".join(single) if single else "Single-pad nets: (none)")
    if no_net_rows:
        no_net_strs = [f"{r['ref']}.{r['pin']} ({r['x']:.3f}, {r['y']:.3f})" for r in no_net_rows]
        lines.append("Pads with no net assigned (not a net -- not electrically joined to each other): " + ", ".join(no_net_strs))

    lines.append("")
    lines.append("=== Summary 2: U1 pad rows ===")
    u1_rows = [r for r in rows if r["ref"] == "U1"]
    if not u1_rows:
        lines.append("U1 not found on board")
    else:
        y_values = sorted(set(round(r["y"], 3) for r in u1_rows))
        for y in y_values:
            row_pads = sorted([r for r in u1_rows if round(r["y"], 3) == y], key=lambda r: r["x"])
            xs = [r["x"] for r in row_pads]
            pitches = sorted(set(round(xs[i + 1] - xs[i], 3) for i in range(len(xs) - 1))) if len(xs) > 1 else []
            lines.append(
                f"row y={y:.3f}mm: {len(row_pads)} pads, first x={xs[0]:.3f}mm ({row_pads[0]['ref']}.{row_pads[0]['pin']}), "
                f"last x={xs[-1]:.3f}mm ({row_pads[-1]['ref']}.{row_pads[-1]['pin']}), pitch(es)={pitches}"
            )

    lines.append("")
    lines.append("=== Summary 3: open front-side routing channel between U1's pad rows ===")
    if len(u1_rows) < 2:
        lines.append("U1 has fewer than 2 distinct pad rows; cannot compute channel")
    else:
        y_values = sorted(set(round(r["y"], 3) for r in u1_rows))
        if len(y_values) != 2:
            lines.append(f"U1 pads span {len(y_values)} distinct Y values, expected 2: {y_values}")
        else:
            top_y, bottom_y = y_values
            top_row = [r for r in u1_rows if round(r["y"], 3) == top_y]
            bottom_row = [r for r in u1_rows if round(r["y"], 3) == bottom_y]
            # inner edge of top row = max(y + h/2) among top row pads
            top_inner_y = max(r["y"] + r["h"] / 2 for r in top_row)
            bottom_inner_y = min(r["y"] - r["h"] / 2 for r in bottom_row)
            all_x = [r["x"] for r in u1_rows]
            chan_x0 = min(all_x)
            chan_x1 = max(all_x)
            lines.append(
                f"channel rectangle: x=[{chan_x0:.3f}, {chan_x1:.3f}]mm, "
                f"y=[{top_inner_y:.3f}, {bottom_inner_y:.3f}]mm "
                f"(height={bottom_inner_y - top_inner_y:.3f}mm)"
            )
            # anything else occupying that rectangle: check other footprints' pads/courtyards
            occupants = []
            for fp in board.GetFootprints():
                ref = fp.GetReference()
                if ref == "U1":
                    continue
                bbox = fp.GetBoundingBox()
                fx0, fy0, fx1, fy1 = mm(bbox.GetLeft()), mm(bbox.GetTop()), mm(bbox.GetRight()), mm(bbox.GetBottom())
                if fx1 < chan_x0 or fx0 > chan_x1 or fy1 < top_inner_y or fy0 > bottom_inner_y:
                    continue
                occupants.append(f"{ref} bbox=({fx0:.3f},{fy0:.3f})-({fx1:.3f},{fy1:.3f})mm")
            if occupants:
                lines.append("occupants already in that rectangle: " + "; ".join(occupants))
            else:
                lines.append("occupants already in that rectangle: (none -- channel is clear)")

    output = "\n".join(lines) + "\n"
    print(output)

    os.makedirs(os.path.dirname(REPORT_PATH), exist_ok=True)
    with open(REPORT_PATH, "w", encoding="utf-8") as f:
        f.write(output)
    print(f"[report] wrote {REPORT_PATH}")


if __name__ == "__main__":
    main()
