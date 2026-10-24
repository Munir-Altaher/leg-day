"""Gymnasium environment: the free leg on its snowshoe learns a full backflip routine (MuJoCo, CPU, no NVIDIA GPU).

Scene: sim/leg_snowshoe_scene.xml (from sim/build_mujoco.py): floating leg, 350 g counterweight, hard floor.
Action (3, in [-1, 1]): joint position targets for ab/ad, hip, knee around the standing pose; a PD loop turns
them into torques at 2 kHz, then the REAL drive limits are applied:
  - ab/ad and hip motors: +-4.1 N m (AK40-10 peak)
  - knee: driven by the knee motor at the hip through the 20T:60T belt -> knee torque = 3 tau_m, belt-limited
    to 2.29 N m (tau_m <= 0.76 N m), and the knee motor's torque tau_m also acts on the hip joint
    (hip joint torque = tau_hip_motor + tau_m, with tau_hip_motor clipped to +-4.1 N m).

The routine, on a fixed clock (the phase is in the observation):
  1. STAND   0-2 s   hold the standing pose, still. Leaving the ground ends the try.
  2. WIND    2-4 s   wind up: sink slowly into a deep crouch while rocking side to side on the ab/ad joint
                     (tracks a +-20 deg roll, two swings), foot planted. Leaving the ground ends the try.
  3. FLIP    4 s ->  jump and turn backwards one full turn in the air (rotation only pays while airborne,
                     only once, and over-rotation past ~380 deg costs). No take-off by 5 s ends the try.
                     Late in the flight the joints go soft and well damped and the knee reaches out.
  4. LAND    first touchdown after the flight: squat (knee folds from its touchdown angle, 0.3 s), then rise
                     back to the standing pose and hold it still until 8 s. Leaving the ground again = fail.
Success ("landed"): 2 s continuously upright, still and in the standing pose after the flip.
Penalties throughout: torque, jerky commands, sitting on a joint stop; a crash (anything but the shoe touching
the floor) ends the try.
History: v1 tumbled over its planted foot; v2's "landings" summed short touches across bounces; v6 touched down
short of 340 deg, rocked back and flipped again for the same reward; v9 strung four 0.03 s hops into one
"flight" and was paid the landing reward without turning. Each is closed off above: the landing phase only
starts after one continuous flight that turned >= 300 deg; any other touchdown after leaving the ground fails.
Backward = the robot's front (+x) goes up = positive rotation about the body Z axis (outboard).
"""
import math
import os
import time

import gymnasium as gym
import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "leg_snowshoe_scene.xml")
TAU_PEAK = 4.1
BELT_TAU = 2.29                      # knee joint torque at 120 N belt tension
N_BELT = 3.0
SPAN = {"abad": 0.78, "hip": 1.6, "knee": 1.2}     # action range around the standing pose (rad); ab/ad: full +-45
KP = {"abad": 20.0, "hip": 25.0, "knee": 25.0}
KD = {"abad": 0.5, "hip": 0.6, "knee": 0.6}
# landing mode: late in the flight the PD loop goes soft and well damped, so the leg gives like a spring-damper
# and soaks up the touchdown instead of bouncing off a stiff leg (on the AK40s: new kp/kd in the MIT-mode command)
LAND_KP, LAND_KD = 0.35, 2.5
CTRL_HZ = 100
T_WIND, T_GO, T_LATE = 2.0, 4.0, 5.0              # wind-up starts, flip allowed, latest take-off
EP_SECONDS = 8.0
CROUCH = 0.3                                      # wind-up crouch past the stance bend (rad); ~17 deg is what
                                                  # the belt-limited knee holds (20 deg sinks and falls over)
ROCK = math.radians(20.0)                         # wind-up ab/ad roll amplitude (tested: stays up)
HOLD = 2.0                                        # seconds stood still after landing = success
STAND, WIND, FLIP, LAND = range(4)
FLIPPED = math.radians(300)                       # turned this far in one flight = the flip counts
FLIP_STARTS = os.path.join(HERE, "runs", "backflip", "flip_starts.npz")   # from sim/make_flip_starts.py


class BackflipEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": CTRL_HZ}

    def __init__(self, render_mode=None, seed=None, flip_start=0.0):
        self.flip_start = flip_start         # share of tries that start wound up at T_GO (training only)
        self.starts = dict(np.load(FLIP_STARTS)) if flip_start > 0 else None
        self.m = mujoco.MjModel.from_xml_path(SCENE)
        self.d = mujoco.MjData(self.m)
        self.render_mode = render_mode
        self.viewer = None
        self.n_sub = int(round(1.0 / CTRL_HZ / self.m.opt.timestep))
        J = lambda n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, n)          # noqa: E731
        self.qa = {n: self.m.jnt_qposadr[J(n)] for n in ("abad", "hip", "knee", "ankle")}
        self.va = {n: self.m.jnt_dofadr[J(n)] for n in ("abad", "hip", "knee", "ankle")}
        self.lim = {n: self.m.jnt_range[J(n)] for n in ("abad", "hip", "knee")}
        self.act = {n: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ("abad", "hip", "knee")}
        self.m.actuator_ctrlrange[self.act["hip"]] = [-(TAU_PEAK + BELT_TAU / N_BELT), TAU_PEAK + BELT_TAU / N_BELT]
        self.base = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "base")
        self.shoe_geoms = {g for g in range(self.m.ngeom) if self.m.body(self.m.geom_bodyid[g]).name == "shoe"}
        self.floor = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
        self.stand = {n: float(self.m.key_qpos[0][self.qa[n]]) for n in ("abad", "hip", "knee")}
        self.h0 = float(self.m.key_qpos[0][2])
        self.action_space = gym.spaces.Box(-1.0, 1.0, (3,), np.float32)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (29,), np.float32)
        self.rng = np.random.default_rng(seed)

    # ------------------------------------------------------------------ drive model
    def _torques(self, target):
        q = lambda n: self.d.qpos[self.qa[n]]               # noqa: E731
        v = lambda n: self.d.qvel[self.va[n]]               # noqa: E731
        kp, kd = (LAND_KP, LAND_KD) if self.soft else (1.0, 1.0)
        des = {n: kp * KP[n] * (target[n] - q(n)) - kd * KD[n] * v(n) for n in target}
        tau_m = np.clip(des["knee"] / N_BELT, -BELT_TAU / N_BELT, BELT_TAU / N_BELT)      # knee motor (belt-limited)
        tau_hm = np.clip(des["hip"] - tau_m, -TAU_PEAK, TAU_PEAK)                       # hip motor
        return {"abad": float(np.clip(des["abad"], -TAU_PEAK, TAU_PEAK)), "hip": float(tau_hm + tau_m),
                "knee": float(N_BELT * tau_m)}

    def _q(self, n):
        return float(self.d.qpos[self.qa[n]])

    def _rock(self, t):
        """Wind-up ab/ad roll target: two side-to-side swings over the 2 s wind-up, back to 0 at the end."""
        return ROCK * math.sin(2 * math.pi * (t - T_WIND) / (T_GO - T_WIND) * 2) if T_WIND <= t < T_GO else 0.0

    def _obs(self):
        d = self.d
        j = [d.qpos[self.qa[n]] for n in ("abad", "hip", "knee", "ankle")]
        jv = [d.qvel[self.va[n]] for n in ("abad", "hip", "knee", "ankle")]
        ph = np.zeros(4)
        ph[self.phase] = 1.0
        return np.concatenate([d.qpos[3:7], d.qvel[3:6], d.qvel[0:3], [d.qpos[2] - self.h0], j, np.array(jv) * 0.1,
                               [self.turn / (2 * math.pi), self.t / EP_SECONDS], self.last, ph,
                               [self._rock(self.t)]]).astype(np.float32)

    def _contacts(self):
        shoe, other = False, False
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if self.floor not in (c.geom1, c.geom2):
                continue
            g = c.geom2 if c.geom1 == self.floor else c.geom1
            if g in self.shoe_geoms:
                shoe = True
            else:
                other = True
        return shoe, other

    def _pose_err(self):
        return sum((self._q(n) - self.stand[n]) ** 2 for n in ("abad", "hip", "knee"))

    # ------------------------------------------------------------------ gym API
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        mujoco.mj_resetDataKeyframe(self.m, self.d, 0)
        for n in ("abad", "hip", "knee"):
            self.d.qpos[self.qa[n]] += self.rng.uniform(-0.02, 0.02)
        mujoco.mj_forward(self.m, self.d)
        self.t, self.turn, self.phase = 0.0, 0.0, STAND
        self.soft, self.td_t, self.knee_td = False, 0.0, None
        self.best, self.air_t, self.stood_t, self.flew = 0.0, 0.0, 0.0, False
        self.last = np.zeros(3)
        self.filt = np.zeros(3)
        if self.starts is not None and self.rng.random() < self.flip_start:
            i = self.rng.integers(len(self.starts["qpos"]))       # skip ahead: wound up, flip allowed now
            self.d.qpos[:], self.d.qvel[:] = self.starts["qpos"][i], self.starts["qvel"][i]
            self.filt, self.last = self.starts["filt"][i].copy(), self.starts["last"][i].copy()
            mujoco.mj_forward(self.m, self.d)
            self.t, self.phase = T_GO, FLIP
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1, 1)
        self.filt = 0.5 * self.filt + 0.5 * a                  # smoothed command (~15 ms): no 100 Hz bang-bang
        target = {n: self.stand[n] + SPAN[n] * self.filt[i] for i, n in enumerate(("abad", "hip", "knee"))}
        turn0 = self.turn
        tq_cost = 0.0
        for _ in range(self.n_sub):
            tq = self._torques(target)
            for n, t in tq.items():
                self.d.ctrl[self.act[n]] = t
            mujoco.mj_step(self.m, self.d)
            self.turn += self.d.qvel[5] * self.m.opt.timestep                 # rotation about body Z (outboard)
            tq_cost += sum(t * t for t in tq.values()) * self.m.opt.timestep
        self.t += self.n_sub * self.m.opt.timestep
        da = a - self.last
        self.last = a
        h = self.d.qpos[2] - self.h0
        up = self.d.xmat[self.base].reshape(3, 3)[:, 1]                       # CAD +Y (up) in world
        shoe, crash = self._contacts()
        airborne = not shoe
        still = float(np.abs(self.d.qvel[0:6]).sum())
        r, terminated = 0.0, False

        # ---- phase clock
        if self.phase == STAND and self.t >= T_WIND:
            self.phase = WIND
        if self.phase == WIND and self.t >= T_GO:
            self.phase = FLIP
        if self.phase == FLIP and airborne:
            self.air_t += 1.0 / CTRL_HZ                         # ONE continuous flight (v9 summed little hops)
            self.flew = self.flew or self.air_t >= 0.12
        flop = False
        if self.phase == FLIP and shoe and self.air_t >= 0.03:  # first touchdown after leaving the ground:
            if self.best >= FLIPPED:                            # the flip is done -> landing
                self.phase = LAND
                self.knee_td = self._q("knee")
            else:
                flop = True                                     # a hop or a short flip: the try failed

        if self.phase in (STAND, WIND):
            if self.air_t > 0 or (airborne and h > 0.01):
                r -= 20.0                                       # jumped early: the routine failed
                terminated = True
            r += 0.5 * up[2] - 0.02 * still                     # upright and calm
            if self.phase == STAND:
                r += 1.0 * math.exp(-self._pose_err() / 0.02)   # hold the standing pose
            else:
                k = (self.t - T_WIND) / (T_GO - T_WIND)         # sink slowly: crouch target ramps in
                knee_goal = self.stand["knee"] + CROUCH * min(1.0, 1.25 * k)
                r += 1.0 * math.exp(-(self._q("knee") - knee_goal) ** 2 / 0.02)
                r += 1.0 * math.exp(-(self._q("abad") - self._rock(self.t)) ** 2 / 0.02)   # rock on ab/ad

        elif self.phase == FLIP:
            new = max(0.0, min(self.turn, 2 * math.pi) - self.best)            # only NEW rotation pays
            self.best = max(self.best, min(self.turn, 2 * math.pi))
            if airborne and h > -0.03:
                r += 12.0 * new                    # rotation only counts in the air (no tumbling over the foot)
                r += 1.0 * max(h, 0.0)             # height while in the air
            if self.air_t < 0.12 and self.d.qvel[2] > 0:
                r += 0.3 * self.d.qvel[2]          # take-off: reward upward speed
            if self.best >= 1.5 * math.pi:         # late in the flight: go soft and reach the knee out, so there
                self.soft = True                   # is travel left to squat into on touchdown
                r -= 3.0 * max(0.0, self._q("knee") - (self.stand["knee"] + 0.15))
                r += 0.5 * up[2]
            if (self.t >= T_LATE and not self.flew) or flop:
                r -= 10.0                          # never jumped, or came down without a full turn
                terminated = True
            elif shoe and self.air_t > 0:
                self.air_t = 0.0                   # contact flicker during push-off: not a take-off yet

        else:                                      # LAND: squat, rise back to the standing pose, hold still
            self.td_t += 1.0 / CTRL_HZ
            r += 0.5 * up[2]
            r -= 3.0 * max(float(self.d.qvel[2]), 0.0) if self.td_t < 0.3 else 0.0     # no rebound
            if self.td_t < 0.3:
                flex = self._q("knee") - self.knee_td           # absorb: fold the knee FROM its touchdown angle
                r += 4.0 * float(np.clip(flex, 0.0, 0.8))       # up to ~45 deg of squat
            else:
                if self.td_t > 0.5:
                    self.soft = False                           # back to normal stiffness to stand up
                pose = math.exp(-self._pose_err() / 0.03)
                ok = shoe and up[2] > 0.85
                r += (2.0 + 3.0 * pose - 0.03 * still) if ok else 0.0    # rise to the standing pose and stay
                self.stood_t = self.stood_t + 1.0 / CTRL_HZ if (ok and pose > 0.5 and still < 1.5) else 0.0
            if airborne:
                r -= 20.0                                       # left the ground again after landing: bounce
                terminated = True

        if self.turn > 2 * math.pi + 0.35:
            r -= 6.0 * max(0.0, self.turn - turn0)              # over-rotation past ~380 deg
        r -= 0.002 * tq_cost
        r -= 0.02 * float(da @ da)                              # smooth commands
        for n in ("abad", "hip", "knee"):                       # stay off the stops
            lo, hi = self.lim[n]
            if not lo + 0.05 <= self._q(n) <= hi - 0.05:
                r -= 0.2
        if crash:
            # falling over after landing is the worst outcome; toppling instead of jumping must not be cheaper
            # than never jumping (the v8 run learned to fall over at 4.7 s to dodge the -10)
            r -= 30.0 if self.phase == LAND else (10.0 if self.phase == FLIP and not self.flew else 3.0)
            terminated = True
        truncated = self.t >= EP_SECONDS
        if self.render_mode == "human":
            self.render()
        return self._obs(), float(r), terminated, truncated, {
            "turn_deg": math.degrees(self.turn), "landed": self.stood_t >= HOLD, "phase": self.phase}

    def render(self):
        if self.viewer is None:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.m, self.d)
            self.viewer.cam.azimuth, self.viewer.cam.elevation, self.viewer.cam.distance = 90.0, -12.0, 1.1
            self.viewer.cam.lookat[:] = [0.0, 0.0, 0.18]
            self._wall = time.perf_counter()
        self.viewer.sync()
        # real time
        self._wall += 1.0 / CTRL_HZ
        dt = self._wall - time.perf_counter()
        if dt > 0:
            time.sleep(dt)
        else:
            self._wall = time.perf_counter()

    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
