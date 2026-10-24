"""Replay a trained backflip policy in the MuJoCo viewer, on a loop (close the window to stop).
  sim/.venv/Scripts/python sim/play_backflip.py [path/to/policy.zip]     (default: sim/runs/backflip/latest.zip)
"""
import os
import sys
import time

from stable_baselines3 import PPO

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from backflip_env import CTRL_HZ, EP_SECONDS, BackflipEnv  # noqa: E402

path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "runs", "backflip", "latest.zip")
model = PPO.load(path)
env = BackflipEnv(render_mode="human")
while True:
    obs, _ = env.reset()
    for _ in range(int(EP_SECONDS * CTRL_HZ)):
        act, _ = model.predict(obs, deterministic=True)
        obs, r, term, trunc, info = env.step(act)
        if term or trunc:
            break
    print(f"turned {info['turn_deg']:.0f} deg, {'LANDED' if info['landed'] else 'no landing'}", flush=True)
    if env.viewer is not None and not env.viewer.is_running():
        break
    time.sleep(0.8)
