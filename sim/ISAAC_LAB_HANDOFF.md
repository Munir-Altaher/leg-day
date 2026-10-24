# Handoff: train the snowshoe leg's backflip in Isaac Lab (GPU)

You are picking up a reinforcement-learning task that has so far run in MuJoCo on a CPU (`sim/backflip_env.py`,
`sim/train_backflip.py`). Port it to Isaac Lab on this machine's NVIDIA GPU, train it with thousands of parallel
copies, and report honestly whether it learns the full routine. Read this file, `sim/README.md`,
`sim/mass_report.md` and `sim/backflip_env.py` before writing any code: the MuJoCo env is the reference
specification for the task, the reward and the drive model.

## The robot (hardware is frozen: do not propose changes to it)
- One 3-DOF quadruped leg (ab/ad roll, hip pitch, knee pitch), three CubeMars AK40-10 motors, PLA parts, standing
  free on a passive **snowshoe** foot. Nothing holds it up: the base is a floating root.
- Model: `sim/leg_snowshoe.urdf` + `sim/meshes/` (metres). **`sim/mass_report.md` is authoritative:**
  1317 g total, including a **350 g counterweight** in the `abad` link (CoM at z ≈ 95 mm). Without it the leg
  cannot stand (its CoM is ~1.5 mm inside the shoe's inboard edge); that was tested exhaustively. Keep it.
- CAD frame: X forward, Y up, Z outboard. Isaac Lab is Z-up, so the root is rotated +90° about X
  (`sim/leg_snowshoe_cfg.py` already does this).
- Joints and limits: abad ±45°, hip ±100°, knee 15–145°, ankle **passive** −52…−11° (lips on the shoe), friction
  0.09 N·m, damping 0.002. The leg stands by leaning on the ankle's −52° lip.
- Standing pose (MuJoCo keyframe): abad 0, hip −40°, knee 92°, ankle −52°, base 202 mm up in sim.
- **Drive model (must be reproduced, it decides what is physically possible):**
  - abad and hip motors: ±4.1 N·m.
  - The knee motor sits at the hip and drives the knee through a 20T:60T belt:
    knee torque = 3·τm with τm clipped to ±0.763 N·m (knee ≤ 2.29 N·m, the belt limit), AND τm also acts on the
    hip joint: hip joint torque = clip(τ_hip_des − τm, ±4.1) + τm. See `_torques()` in `sim/backflip_env.py`.
  - Implement this as explicit torque control (compute PD torques yourself every physics step, apply the coupling
    and clips, then set joint efforts), not as Isaac's implicit PD per joint, which cannot couple the hip and knee.
  - PD gains: KP 20/25/25, KD 0.5/0.6/0.6 (abad/hip/knee). Policy action in [−1, 1]³ = position target around the
    standing pose, span 0.78/1.6/1.2 rad, low-pass filtered (filt = 0.5·filt + 0.5·a), policy at 100 Hz, physics
    2 kHz in MuJoCo (use ≥ 1 kHz in PhysX).
  - **Soft landing mode:** late in the flight the gains become KP×0.35, KD×2.5, so the leg gives like a
    spring-damper on touchdown (on the real AK40s this is just new kp/kd in the MIT-mode command).
- **Contact:** in MuJoCo the shoe touches the floor through 19 small spheres (r 1.5 mm) around its sole outline
  (`sim/leg_snowshoe_scene.xml`, geoms `sole0…18`); other bodies use their meshes. Floor friction 0.6, hard floor.
  The thin shoe mesh is a poor PhysX collider, so give the shoe primitive colliders (spheres or a thin box) in the
  same places, and make "anything other than the shoe touching the floor" detectable (contact sensors).

## The task: one fixed-clock routine, 8 s per episode (phase is in the observation)
1. **STAND 0–2 s:** hold the standing pose, still. Leaving the ground ends the try (−20).
2. **WIND 2–4 s:** sink into a crouch (knee +0.3 rad over stance, ramped in; deeper collapses under the belt
   limit) while rocking the ab/ad joint ±20° (two sine swings), foot planted.
3. **FLIP from 4 s:** jump and rotate **backwards one full turn** (positive rotation about the body Z axis =
   the front goes up) in a single flight. No take-off by 5 s = fail (−10).
4. **LAND:** first touchdown after that flight: squat to absorb the impact (knee folds from its touchdown angle,
   0.3 s), then stand back up to the standing pose and hold still until 8 s. Leaving the ground again = fail.
- **Success** = after the flip, 2 s continuously upright, still and in the standing pose.

## Reward loopholes already found (do not reopen them)
- v1: "rotated" by tumbling over the planted foot → rotation only pays while airborne.
- v2: "landings" were summed short touches across bounces → stand time must be continuous; a bounce ends the try.
- v6: touched down short, rocked back and flipped again → rotation pays only once (track the best angle),
  over-rotation past ~380° is penalised.
- v8: fell over on purpose at 4.7 s because crashing (−3) was cheaper than not jumping (−10) → a crash before
  the flight now costs −10.
- v9: strung four 0.03 s hops into one "flight" and got the landing reward without rotating → the landing phase
  needs ONE continuous flight that turned ≥ 300°; any other touchdown after leaving the ground fails (−10).
- Policy exploration std collapsed to ~0.01 in long runs and the policy stopped trying new moves; watch it.
- Half the training tries start already wound up at t = 4 s (`sim/make_flip_starts.py`,
  `runs/backflip/flip_starts.npz`) so the flip gets practised. In Isaac you can do the same from a state saved
  out of your own wind-up, or reset a share of envs straight into a crouched pose at t = 4 s.

## Status in MuJoCo (CPU, ~2000 steps/s) when this was handed over
It stands and does the wind-up reliably. It has **not** yet produced a real backflip with this routine; earlier,
simpler setups did rotate ~340–360° but did not land. The last run was retrained after the hop loophole was closed.

## What to do
1. Set up: confirm the GPU, Isaac Sim and Isaac Lab versions; install anything missing in a venv/conda env.
   Use the Isaac Lab version's own APIs (`sim/leg_snowshoe_cfg.py` was written for Isaac Lab 2.x and never run:
   check every class/field against the installed version).
2. Import the URDF, check the mass (1317 g), joint limits and standing pose, and verify **before any RL** that
   with the PD + coupled drive model holding the standing pose the leg stands still for 5 s on all envs (the
   MuJoCo check is `sim/build_mujoco.py --check`). Also check the 0.3 rad crouch + ±20° rock stays up.
3. Write a `DirectRLEnv` (manager-based is fine too) reproducing `sim/backflip_env.py`: observation (29 values),
   action, phases, reward terms and terminations. Keep the same numbers unless you find a reason, and write down
   every change and why.
4. Train with RSL-RL or rl_games PPO, 2048–4096 envs, headless. Log turn angle, take-off, touchdown and success
   rates per phase, not only mean reward.
5. Evaluate honestly on ≥ 100 episodes with a deterministic policy: where do tries end (phase + reason), max turn
   in one flight, landing success by the strict definition above. Inspect a few episodes frame by frame; a high
   reward has been a loophole five times already.
6. Record a video of the best policy doing the whole routine, and export the policy (TorchScript/ONNX) plus a
   short note on observation/action scaling, so it can be run on the hardware later.
7. Commit your work on a new branch in this repo (e.g. `isaac-lab-backflip`) with a README section on how to run
   it; do not push to `main`.

## Constraints
- Do not change the hardware: the parts are printed. A counterweight of 350 g is the accepted fix.
- Keep the real drive limits; do not let the sim use torque the motors or the belt don't have.
- Report what did and didn't work with numbers. If the backflip is physically out of reach with these torques,
  say so and show why (e.g. maximum achievable take-off speed and spin rate vs what one turn needs).
