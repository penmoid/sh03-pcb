"""check_determinism.py - acceptance test for the sh03 PCB generation pipeline.

Runs place_footprints.py -> route_board.py -> add_ground_pour.py ->
add_fiducials.py twice from the same starting board state, hashing the
resulting .kicad_pcb after each run. Reports both hashes and whether they
match.

Run with the SAME interpreter as the pipeline scripts (they import pcbnew --
this script doesn't need to, but uses sys.executable to invoke the four
pipeline stages, so it must itself be launched with KiCad's bundled python):
  python3 scripts/check_determinism.py

Requires: build/sh03.net already exported (see place_footprints.py's
docstring) -- none of the four pipeline scripts touch it, so it's identical
across both runs by construction.

Safety: this tool only *checks* things, so it must never discard work as a
side effect. Before doing anything else it refuses to run if
sh03-controller.kicad_pcb has uncommitted changes, unless overridden with
--force (in which case it prints what it's about to do, same as without
--force, just doesn't stop). Either way, the board file present at start is
copied aside once and copied back once the run is done, restoring it byte-
for-byte -- never a `git checkout`, which is what silently discarded
uncommitted board work the last time this tool ran on a dirty tree. The
restore is wrapped in try/finally so it happens on every exit path,
including an unhandled exception or Ctrl-C.

Exit code 0 on match, 1 on mismatch or a pipeline stage failure, 2 if the
tree is dirty and --force wasn't given.
"""

import argparse
import hashlib
import os
import shutil
import subprocess
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
PCB_PATH = os.path.join(PROJECT_DIR, "sh03-controller.kicad_pcb")
PCB_RELPATH = "sh03-controller.kicad_pcb"
NET_PATH = os.path.join(PROJECT_DIR, "build", "sh03.net")
BACKUP_PATH = PCB_PATH + ".determinism-backup"

PIPELINE = ["place_footprints.py", "route_board.py", "add_ground_pour.py", "add_fiducials.py"]

PYTHON = sys.executable  # must be KiCad's bundled interpreter -- see docstring


def board_is_dirty():
    """True if sh03-controller.kicad_pcb has uncommitted changes (staged or
    unstaged) relative to HEAD."""
    result = subprocess.run(
        ["git", "diff", "--quiet", "HEAD", "--", PCB_RELPATH],
        cwd=PROJECT_DIR,
    )
    return result.returncode != 0


def run_stage(script_name):
    script_path = os.path.join(SCRIPT_DIR, script_name)
    result = subprocess.run(
        [PYTHON, script_path], cwd=PROJECT_DIR,
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        raise SystemExit(f"[determinism] {script_name} exited {result.returncode}")
    return result.stdout


def sha256_of(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        h.update(f.read())
    return h.hexdigest()


def run_chain(label):
    shutil.copyfile(BACKUP_PATH, PCB_PATH)  # start every run from the same board state
    for script_name in PIPELINE:
        run_stage(script_name)
    digest = sha256_of(PCB_PATH)
    print(f"[determinism] {label}: sha256={digest}")
    return digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--force", action="store_true",
        help="run even if sh03-controller.kicad_pcb has uncommitted changes "
             "(the board is still saved and restored, not discarded)",
    )
    args = parser.parse_args()

    if not os.path.isfile(NET_PATH):
        raise SystemExit(f"[determinism] netlist not found: {NET_PATH} -- export it first")

    dirty = board_is_dirty()
    if dirty and not args.force:
        raise SystemExit(
            f"[determinism] {PCB_RELPATH} has uncommitted changes.\n"
            "This tool overwrites it repeatedly while testing the pipeline. "
            "Your uncommitted edits ARE preserved (saved aside and restored when "
            "the tool exits) -- but running with a dirty tree unreviewed is "
            "usually a mistake, so it's refused by default.\n"
            "Commit or stash your changes, or re-run with --force to proceed "
            "anyway."
        )
    if dirty:
        print(f"[determinism] {PCB_RELPATH} has uncommitted changes -- proceeding "
              "under --force. They will be saved aside now and restored when this "
              "tool exits, not discarded.")

    print(f"[determinism] python: {PYTHON}")
    print(f"[determinism] pipeline: {' -> '.join(PIPELINE)}")
    print()

    shutil.copyfile(PCB_PATH, BACKUP_PATH)
    try:
        digest_a = run_chain("run A")
        digest_b = run_chain("run B")

        print()
        match = digest_a == digest_b
        if match:
            print(f"[determinism] MATCH: both runs produced sha256={digest_a}")
        else:
            print("[determinism] MISMATCH:")
            print(f"  run A: {digest_a}")
            print(f"  run B: {digest_b}")
    finally:
        # restore on every exit path -- normal return, SystemExit from a
        # failed stage, or any other exception (e.g. Ctrl-C)
        shutil.copyfile(BACKUP_PATH, PCB_PATH)
        os.remove(BACKUP_PATH)

    sys.exit(0 if match else 1)


if __name__ == "__main__":
    main()
