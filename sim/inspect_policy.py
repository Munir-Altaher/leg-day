"""Replay a saved policy headless and log what the leg does (joint angles, height, rotation, contacts).
  sim/.venv/Scripts/python sim/inspect_policy.py [policy.zip]
"""
import math
import os
import sys

import numpy as np
from stable_baselines3 import PPO

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from backflip_env import BackflipEnv  # noqa: E402

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "runs", "backflip", "latest.zip")
    model = PPO.load(path, device="cpu")
    e = BackflipEnv(seed=0)
    obs, _ = e.reset(seed=0)
    m, d = e.m, e.d
    deg = lambda n: math.degrees(d.qpos[e.qa[n]])               # noqa: E731
    print("  t     turn  height  hip   knee  ankle  abad   act(abad,hip,knee)      floor contacts")
    for k in range(200):
        a, _ = model.predict(obs, deterministic=True)
        obs, r, term, trunc, info = e.step(a)
        if k % 5 == 0 or term or trunc:
            who = sorted({m.body(m.geom_bodyid[c.geom2 if c.geom1 == e.floor else c.geom1]).name
                          for c in (d.contact[i] for i in range(d.ncon)) if e.floor in (c.geom1, c.geom2)})
            print(f"{e.t:5.2f} {info['turn_deg']:7.0f} {1000 * (d.qpos[2] - e.h0):6.0f} {deg('hip'):6.1f} {deg('knee'):6.1f} "
                  f"{deg('ankle'):6.1f} {deg('abad'):5.1f}   {np.round(a, 2)}   {who}")
        if term or trunc:
            print("ended:", "terminated" if term else "time limit", "landed" if info["landed"] else "")
            break
