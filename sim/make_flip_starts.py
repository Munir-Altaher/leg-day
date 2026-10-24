"""Record wound-up states to start practice tries from, right at the moment the flip is allowed (t = T_GO).

  sim/.venv/Scripts/python sim/make_flip_starts.py [policy.zip]     -> sim/runs/backflip/flip_starts.npz

A policy that already stands and winds up is run to t = T_GO on many seeds; every try still upright on its shoe
is saved. Training (BackflipEnv(flip_start=p)) then starts a fraction p of its tries from one of these, so the
flip gets practised every try instead of only after 4 s of standing (reference-state initialisation).
"""
import os
import sys

import numpy as np
from stable_baselines3 import PPO

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from backflip_env import CTRL_HZ, FLIP_STARTS, T_GO, BackflipEnv  # noqa: E402

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "runs", "backflip", "latest.zip")
    model = PPO.load(path, device="cpu")
    rec = {k: [] for k in ("qpos", "qvel", "filt", "last")}
    for s in range(60):
        e = BackflipEnv(seed=1000 + s)
        o, _ = e.reset(seed=1000 + s)
        for _ in range(int(round(T_GO * CTRL_HZ)) - 1):
            o, r, te, tr, i = e.step(model.predict(o, deterministic=False)[0])   # some spread between starts
            if te or tr:
                break
        shoe, crash = e._contacts()
        if te or tr or crash or not shoe:
            continue
        for k, v in (("qpos", e.d.qpos), ("qvel", e.d.qvel), ("filt", e.filt), ("last", e.last)):
            rec[k].append(np.array(v))
    np.savez(FLIP_STARTS, **{k: np.array(v) for k, v in rec.items()})
    print(f"saved {len(rec['qpos'])} wound-up starts to {FLIP_STARTS}")
