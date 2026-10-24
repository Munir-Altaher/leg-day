"""Regenerate the whole snow leg from params.py, in dependency order, and stop on the first failure.
Everything is written inside snow_leg/ (all paths are relative to the scripts).

  python regenerate.py            full chain (SolidWorks must be installed; ~30-40 min)
  python regenerate.py --analysis analysis only (no SolidWorks)

Chain: params -> leg_model -> snow_model -> build_parts -> clearance_check (re-solves the ab/ad stop
blocks) -> build_parts body_spine (with the blocks) -> build_assembly -> verify_mates -> print_check
(print-oriented STLs + PrusaSlicer) -> insert_check -> skeleton_check -> gait_check -> walk_animation.
"""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
ENV = dict(os.environ, PYTHONIOENCODING="utf-8")

ANALYSIS = [["params.py"], ["analysis/leg_model.py"], ["analysis/snow_model.py"]]
CAD = [["cad/build_parts.py"], ["analysis/clearance_check.py"], ["cad/build_parts.py", "body_spine"],
       ["cad/build_assembly.py"], ["cad/verify_mates.py"], ["analysis/print_check.py"]]
POST = [["analysis/insert_check.py"], ["analysis/skeleton_check.py"], ["analysis/gait_check.py"]]
ANIM = [["cad/walk_animation.py"]]


def run(steps, log):
    for st in steps:
        t0 = time.time()
        r = subprocess.run([PY] + [os.path.join(HERE, st[0])] + st[1:], cwd=HERE, env=ENV, capture_output=True, text=True, encoding="utf-8", errors="replace")
        log.write(f"\n===== {' '.join(st)}  (exit {r.returncode}, {time.time() - t0:.0f} s)\n{r.stdout[-6000:]}\n{r.stderr[-3000:]}\n")
        log.flush()
        print(f"{' '.join(st):40s} exit {r.returncode}  {time.time() - t0:6.0f} s", flush=True)
        if r.returncode != 0 and st[0] not in ("analysis/clearance_check.py", "analysis/insert_check.py", "analysis/snow_model.py"):
            return False
    return True


if __name__ == "__main__":
    with open(os.path.join(HERE, "analysis", "out", "regenerate_log.txt"), "w", encoding="utf-8") as log:
        steps = ANALYSIS + ([] if "--analysis" in sys.argv else CAD) + POST + ([] if "--analysis" in sys.argv or "--no-anim" in sys.argv else ANIM)
        ok = run(steps, log)
    print("REGENERATE", "OK" if ok else "STOPPED")
