"""Train the snowshoe leg to backflip with PPO, and watch it learn.

  sim/.venv/Scripts/python sim/train_backflip.py --watch            # train; after every round the 3D viewer
                                                                     # replays the current policy in real time
  sim/.venv/Scripts/python sim/train_backflip.py --steps 3000000    # longer run, no window (faster)
  sim/.venv/Scripts/python sim/play_backflip.py                     # replay the latest saved policy on a loop

Checkpoints and the learning log go to sim/runs/backflip/. Training runs on the CPU: the physics (MuJoCo) is CPU-only
here and the policy network is tiny, so a GPU would not help. Use  --procs --envs 12  to spread the copies of the leg
over the CPU cores (one process each).
"""
import argparse
import os
import sys
import time

import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv, VecMonitor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from backflip_env import CTRL_HZ, EP_SECONDS, BackflipEnv  # noqa: E402

RUN = os.path.join(HERE, "runs", "backflip")


class Progress(BaseCallback):
    """Prints how the policy is doing each round, saves it, and (with --watch) replays it in the viewer."""

    def __init__(self, watch, every):
        super().__init__()
        self.watch, self.every, self.round = watch, every, 0
        self.viewer_env = BackflipEnv(render_mode="human") if watch else None
        self.t0 = time.time()

    def _on_step(self):
        return True

    def _on_rollout_end(self):
        self.round += 1
        infos = self.model.ep_info_buffer
        mean_r = np.mean([e["r"] for e in infos]) if infos else float("nan")
        self.model.save(os.path.join(RUN, "latest"))
        msg = f"round {self.round:4d}  steps {self.num_timesteps:8d}  mean episode reward {mean_r:8.2f}  ({time.time() - self.t0:5.0f} s)"
        if self.viewer_env is not None and self.round % self.every == 0:
            turn, landed = self.demo()
            msg += f"  | replay: turned {turn:6.0f} deg, {'LANDED' if landed else 'no landing'}"
        print(msg, flush=True)

    def demo(self):
        env = self.viewer_env
        obs, _ = env.reset()
        info = {"turn_deg": 0.0, "landed": False}
        for _ in range(int(EP_SECONDS * CTRL_HZ)):
            act, _ = self.model.predict(obs, deterministic=True)
            obs, r, term, trunc, info = env.step(act)
            if term or trunc:
                break
        return info["turn_deg"], info["landed"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2_000_000)
    ap.add_argument("--watch", action="store_true", help="replay the current policy in the MuJoCo viewer")
    ap.add_argument("--every", type=int, default=1, help="replay every N rounds (with --watch)")
    ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--flip-start", type=float, default=0.0,
                    help="share of tries that start wound up at the flip (needs sim/make_flip_starts.py first)")
    ap.add_argument("--explore", type=float, default=None,
                    help="with --resume: raise the exploration noise std to at least this (e.g. 0.6)")
    ap.add_argument("--procs", action="store_true", help="run each copy of the leg in its own process (uses all CPU cores)")
    a = ap.parse_args()
    os.makedirs(RUN, exist_ok=True)
    if os.name == "nt":                     # keep Windows from idle-sleeping while this process trains (released
        import ctypes                       # when it exits); a sleep broke the worker pipes of the v8 run
        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000 | 0x00000001)
    fns = [lambda i=i: BackflipEnv(seed=i, flip_start=a.flip_start) for i in range(a.envs)]
    venv = VecMonitor(SubprocVecEnv(fns, start_method="spawn") if a.procs else DummyVecEnv(fns))
    latest = os.path.join(RUN, "latest.zip")
    if a.resume and os.path.exists(latest):
        model = PPO.load(latest, env=venv)
        if a.explore:                       # a resumed policy has narrowed its noise; widen it to find new moves
            ls = model.policy.log_std.data
            ls.clamp_(min=float(np.log(a.explore)))
            print("exploration std now", np.round(ls.exp().numpy(), 2), flush=True)
    else:
        model = PPO("MlpPolicy", venv, n_steps=512, batch_size=1024, n_epochs=8, learning_rate=3e-4, gamma=0.99,
                    gae_lambda=0.95, ent_coef=0.005, clip_range=0.2, verbose=0,
                    # exploration noise starts at std 0.37 (not 1.0 = the whole action range): with full-range noise
                    # most early tries fell over in the first second and never practised the later phases
                    policy_kwargs=dict(net_arch=[128, 128], log_std_init=-1.0))
    model.learn(total_timesteps=a.steps, callback=Progress(a.watch, a.every), reset_num_timesteps=not a.resume)
    model.save(os.path.join(RUN, "final"))
    print("saved", os.path.join(RUN, "final.zip"))


if __name__ == "__main__":
    main()
