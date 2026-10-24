"""Simulation speed: all legs on one core vs one process per leg.  sim/.venv/Scripts/python sim/bench_cores.py"""
import os
import sys
import time

import numpy as np
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from backflip_env import BackflipEnv  # noqa: E402


def bench(v, n, sec=6.0):
    v.reset()
    k, t = 0, time.time()
    while time.time() - t < sec:
        v.step(np.random.uniform(-1, 1, (n, 3)).astype(np.float32))
        k += n
    return k / (time.time() - t)


if __name__ == "__main__":
    d = DummyVecEnv([lambda i=i: BackflipEnv(seed=i) for i in range(8)])
    a = bench(d, 8)
    d.close()
    s = SubprocVecEnv([lambda i=i: BackflipEnv(seed=i) for i in range(12)], start_method="spawn")
    b = bench(s, 12)
    s.close()
    print(f"one core, 8 legs:        {a:7.0f} steps/s")
    print(f"12 processes, 12 legs:   {b:7.0f} steps/s  ({b / a:.1f}x)")
