# Simulation model: free-standing leg on the snowshoe

| File | What |
|---|---|
| `export_urdf.py` | regenerates everything below from the CAD (`cad/stl/`) and the slicer weights (`analysis/out/print.json`) |
| `leg_snowshoe.urdf` + `meshes/` | 5 bodies, 4 joints; meshes in metres, CAD body frame |
| `mass_report.md` | mass, centre of mass and joint limits per body |
| `leg_snowshoe_cfg.py` | Isaac Lab `ArticulationCfg` (Isaac Lab 2.x API; check names against your version) |

Regenerate after any CAD change: `python cad/build_parts.py` → `python sim/export_urdf.py`.

## Model
- **Bodies:**
  - `base`: brackets, spine, rig plate and ab/ad motor (304 g, floating root).
  - `abad`: hip module with the hip and knee motors and the 350 g counterweight (846 g).
  - `thigh`: 100 g.
  - `shank`: 33 g.
  - `shoe`: 33 g.
  - **Total: 1317 g** (see `mass_report.md`, which is authoritative).
- **Joints:**
  - `abad` about X, ±45°, 4.1 N·m.
  - `hip` about Z, ±100°, 4.1 N·m.
  - `knee` about Z, 15–145°, **2.29 N·m**, limited by the belt rather than the motor.
  - `ankle` about Z, **passive**: −52…−11° (the lips), friction 0.09 N·m.
- **Frames:** the CAD frame is X forward, Y up, Z outboard. The config rotates the root +90° about X so the model stands Z-up.
- **Standing pose:** q1 = −40°, q2 = 92°, ankle = −52°, with the shoe flat and the hip 186 mm above the ground. The leg leans on the **rear lip**. The ankle is a free friction hinge (0.09 N·m) and the centre of mass sits about 40 mm behind the pivot in a normal stance. Without leaning on a stop, the leg swings back about the ankle and falls; MuJoCo showed exactly this.
- **Hard floor only:** with the 150 g counterweight the centre of mass is about 12 mm inside the shoe's inboard edge. That's enough on a hard floor or table. On soft contact (snow), the inboard edge sinks and the leg rolls over (also checked in MuJoCo).

## Knee drive coupling (needed for sim-to-real)
The knee motor is at the hip and drives the knee through a 20T:60T belt. Its angle is θm = q1 + 3·q2, so a knee-motor torque τm gives:
- knee joint torque = 3·τm, capped by the belt at 2.29 N·m peak (1.15 N·m continuous);
- hip joint torque = τ_hip_motor + τm.

The config limits each joint on its own. Before running anything on the real leg, map the policy's joint torques to motor torques with τm = τ_knee / 3 and τ_hip_motor = τ_hip − τm, then clip each to ±4.1 N·m. Otherwise the simulator lets the hip use torque the real motor doesn't have.

## Found while building this: the leg cannot stand still on the snowshoe sideways
- **The problem:** in the standing pose the whole leg's centre of mass is 20 mm outboard of the ab/ad axis. The shoe spans 18–88 mm (centred on the leg plane at 53 mm), so the centre of mass is only **2 mm inside the shoe's inboard edge**. The heavy parts (ab/ad motor, brackets, spine, hip module) are all inboard.
- **No joint can fix it:** the ab/ad joint moves the centre of mass by only about 1 mm, and the ankle only pivots front to back.
- **Front to back is fine:** the centre of mass is 42 mm behind the ankle, inside the shoe (−66…+44 mm).
- **Fix options:**
  - (a) widen the shoe inboard, making it asymmetric;
  - (b) mount the shoe off-centre, inboard, so its middle sits under the centre of mass;
  - (c) keep the hardware as is and support the leg sideways for the demo, for example with a vertical guide rail or a tether.

## Backflip feasibility (estimate; the simulator will refine it)
- **Weight and push:** the free leg weighs about 9.5 N. The foot can push about 31 N at peak (belt-limited), roughly 3.3 times its weight.
- **Jump:** about 0.1 m of push gives a take-off speed of about 2.2 m/s and roughly 0.45 s in the air.
- **Spin:** a full flip needs about 14 rad/s of spin, which has to come from pushing with the foot ahead of the centre of mass. Swinging the hip in the air only adjusts the angle a little.
- **Belt:** keep peak belt force under 120 N in the reward or limits. A slipping belt ends the move on the real hardware.
- **Practice safely:** use a foam mat and a slack tether.

## MuJoCo: see it and train it on this laptop (no NVIDIA GPU needed)
Isaac Sim needs an NVIDIA RTX GPU; this laptop has AMD Radeon graphics. MuJoCo runs here on the CPU. Everything is installed in `sim/.venv` (MuJoCo 3.17, Gymnasium, Stable-Baselines3, PyTorch CPU). Run these from `leg_day\`:

| What | Command |
|---|---|
| Rebuild the scene after a CAD or URDF change | `sim/.venv/Scripts/python sim/build_mujoco.py --check` |
| Look at the leg (3D viewer: drag to rotate, right-drag to pan, scroll to zoom, space to pause) | `sim/.venv/Scripts/python -m mujoco.viewer --mjcf=sim/leg_snowshoe_scene.xml` |
| Train and watch: the viewer replays the current policy every few rounds | `sim/.venv/Scripts/python sim/train_backflip.py --watch --every 3` |
| Train faster with no window | `sim/.venv/Scripts/python sim/train_backflip.py --steps 5000000` |
| Carry on from the last save | add `--resume` |
| Replay the latest policy on a loop | `sim/.venv/Scripts/python sim/play_backflip.py` |

- **The task** (`backflip_env.py`):
  - The policy sets target angles for ab/ad, hip and knee.
  - Torques respect the real drive: 4.1 N·m motors, the 2.29 N·m belt-limited knee, and the knee motor also loading the hip.
  - Reward: backward rotation (up to one full turn), height in the air, and an upright landing held for 0.3 s.
  - An episode ends if anything but the shoe hits the floor.
- **Logs and saves:** `sim/runs/backflip/` holds `latest.zip` (saved every round) and `final.zip`.
- **Contact model notes:**
  - The sole touches the floor through 19 small spheres on the real sole outline. MuJoCo's mesh-to-floor contact made the shoe behave as if it were narrower.
  - The ankle lips are stiff limits; MuJoCo's default soft limits let gravity push 24° past them.
  - The floor is hard (a table or floor). On soft ground the leg tips sideways.
- **Counterweight: 350 g** on the C-frame's outboard face, the default in `export_urdf.py`. In MuJoCo it stands in 10 of 10 runs, against 8 of 10 with 250 g and 2 of 10 with 150 g.
