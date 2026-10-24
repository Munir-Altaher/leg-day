"""LEG DAY â€” single source of truth. All lengths in mm, angles in degrees.

Body/CAD frame (brief section 4.1): origin = hip-pitch axis âˆ© ab/ad axis.
X forward (= ab/ad axis), Y up, Z lateral (+ outboard). Leg plane (foot/shank
mid-plane) is z = L_ABAD. Every CAD part is generated in this frame at
q0 = q1 = q2 = 0 (leg hanging straight down), so an assembly built from
components inserted at the origin is already in its home pose.

Run `python params.py` to rewrite params.json.
"""
import json
import math
import os

# ---------------------------------------------------------------- switches
SKELETON = os.environ.get("LEG_SKELETON", "1") == "1"   # lightening through-cuts in thigh + shank (section 5.1)
MOTOR_OUTPUT_FACE = "right_inner"   # Ã˜27 3xM2.5 + Ã˜15 boss = rotor (ASSUMED, verify by hand)

# ---------------------------------------------------------------- motor AK40-10 (sheet)
MOTOR_D = 53.0
MOTOR_L = 37.0
MOTOR_MASS = 0.185          # kg (sheet; catalog 0.200)
TAU_RATED = 1.3             # Nm at output
TAU_PEAK = 4.1
SPEED_RATED = 370.0         # rpm
KT_OUT = 0.56               # Nm/A at output
BACKLASH_ARCMIN = 18.0
ROTOR_INERTIA = 540e-7      # kg m^2 (catalog 540 g cm^2 â€” VERIFY, may be pre-gearbox)
GEAR = 10.0
MOTOR_OUT_PCD = 27.0        # rotor 3 x M2.5, 3 mm deep
MOTOR_OUT_HOLE_DEPTH = 3.0
MOTOR_BOSS_D = 15.0
MOTOR_BOSS_H = 1.0          # ASSUMED boss/step proud of face
MOTOR_BACK_PCD = 45.0       # stator 5 x M2.5, 5 mm deep (left/back face)
MOTOR_BACK_N = 5
MOTOR_BACK_HOLE_DEPTH = 5.0

# ---------------------------------------------------------------- link + belt design (analysis section 4.6)
L1 = 130.0                  # hip axis -> knee axis
L2 = 130.0                  # knee axis -> foot sphere centre
GT2_PITCH = 2.0
BELT_W = 6.0
HIP_TEETH = 20              # kit pulley
KNEE_TEETH = 60             # printed (48T gave 9.9 N < 15 N target, see report)
N_RATIO = KNEE_TEETH / HIP_TEETH
PD_HIP = HIP_TEETH * GT2_PITCH / math.pi        # 12.73
PD_KNEE = KNEE_TEETH * GT2_PITCH / math.pi      # 30.56
GT2_PLD = 0.254             # pitch-line offset
OD_HIP = PD_HIP - 2 * GT2_PLD
OD_KNEE = PD_KNEE - 2 * GT2_PLD
GT2_TOOTH_TWEAK = 0.0       # +/- mm on printed tooth width (calibrate with coupon)
BELT_T_PEAK = 120.0         # N effective tension, absolute (firmware limit)
BELT_T_CONT = 60.0          # N continuous
BELT_SUPPLIED = 450.0       # ASSUMED
STANCE_FRAC = 0.82          # nominal stance height / (L1+L2)

# ---------------------------------------------------------------- bearings / axles
B625 = dict(d=5.0, D=16.0, B=5.0)
AXLE_D = 5.0                # steel, M5 bolt shank or D-shaft
FIT_BEARING = 0.10          # added to bearing OD
FIT_LOCATE = 0.20           # Ã˜15 boss pocket etc.
FIT_RUN = 0.35
GAP_MOVING = 0.5            # per side between moving parts
CLR = {"M2.5": 2.9, "M3": 3.4, "M4": 4.5, "M5": 5.5}
TAP = {"M2.5": 2.2, "M3": 2.6}
INSERT_M3 = 4.0
NUT_AF = {"M2.5": 5.2, "M3": 5.7, "M5": 8.2}

# ---------------------------------------------------------------- lateral stack (module frame, z_m = 0 at leg plane)
SHANK_T = 10.0              # shank plate thickness, centred on leg plane (8 mm failed lateral stiffness)
THIGH_WALL = 5.0            # side wall at the bearing/motor bosses (holds a 625 flush)
THIGH_WALL_THIN = 3.0       # side wall elsewhere (thinned from the channel side)
THIGH_BOSS_R_HIP = 20.0     # full-thickness zone around the hip axis
THIGH_BOSS_R_KNEE = 12.0    # full-thickness ring around the knee bearing
THIGH_CHANNEL = 9.0         # belt 6 + 1.5 per side
ADAPTER_T = 6.0             # knee-motor pulley adapter (2.5 cbore + 3.5 under head -> M2.5x6 = 2.5 engagement)
PLATE_T = 5.0               # module side plates (M2.5x8 -> 3.0 engagement in 5 mm holes)
FLANGE_T = 5.5              # ab/ad flanges on rotor (M2.5x8 -> 2.5 engagement in 3 mm holes)
BRACKET_T = 6.0             # body bracket walls
ABAD_CLEAR = 2.0            # ab/ad motor to thigh swept slab

Z_SHANK = (-SHANK_T / 2, SHANK_T / 2)
_z = -SHANK_T / 2 - GAP_MOVING
Z_THIGH_OUT = (_z - THIGH_WALL, _z)                       # wall that bolts to hip motor
Z_CHANNEL = (Z_THIGH_OUT[0] - THIGH_CHANNEL, Z_THIGH_OUT[0])
Z_THIGH_IN = (Z_CHANNEL[0] - THIGH_WALL, Z_CHANNEL[0])    # cover, carries hip-shaft 625
Z_ADAPTER = (Z_THIGH_IN[0] - GAP_MOVING - ADAPTER_T, Z_THIGH_IN[0] - GAP_MOVING)
Z_KNEE_MOTOR = (Z_ADAPTER[0] - MOTOR_L, Z_ADAPTER[0])
Z_PLATE_IN = (Z_KNEE_MOTOR[0] - PLATE_T, Z_KNEE_MOTOR[0])
Z_HIP_MOTOR = (Z_THIGH_OUT[1], Z_THIGH_OUT[1] + MOTOR_L)  # output face on thigh outer wall
Z_PLATE_OUT = (Z_HIP_MOTOR[1], Z_HIP_MOTOR[1] + PLATE_T)

# ab/ad axis: inboard of the thigh slab by motor radius + clearance
L_ABAD = round(-Z_THIGH_IN[0] + MOTOR_D / 2 + ABAD_CLEAR, 1)   # = l, ab/ad axis -> leg plane


def zc(zm):
    """module z (leg plane = 0) -> CAD/body z (ab/ad axis = 0)."""
    return zm + L_ABAD


# ---------------------------------------------------------------- thigh profile
THIGH_R_HIP = 23.0          # outer radius at hip end (covers Ã˜27 PCD + heads)
THIGH_R_KNEE = PD_KNEE / 2 + 7.0
THIGH_CH_R_HIP = 17.5       # channel inner radius at hip (screw heads inside)
THIGH_CH_R_KNEE = PD_KNEE / 2 + 3.0
IDLER_BOLT = "M3"
IDLER_Y = -40.0             # idler slot centre (below hip)
IDLER_SLOT_TRAVEL = 9.0     # >= 8 mm

# ---------------------------------------------------------------- shank + foot
SHANK_W_KNEE = 26.0
SHANK_W_FOOT = 22.0         # wide at the foot end (Spot-like), room for three equal lightening holes
# Spot-style bent shank: one gentle bow, convex to the front (+x, the side the knee folds toward), as on
# Spot's lower leg. The centre line is a cubic Bezier from the knee to the foot plug: it leaves the knee
# |SHANK_BEND_DEG| forward and reaches the foot heading |SHANK_FOOT_TILT_DEG| back; plug and foot are tilted
# to match. Negative angles = bow to +x. The foot centre stays L2 straight below the knee, so the
# kinematics are unchanged. Bows ~9 mm off the knee-foot line.
SHANK_BEND_DEG = -10.0
SHANK_FOOT_TILT_DEG = -10.0
SHANK_BEND_REACH = 45.0     # knee-end Bezier handle length, mm
SHANK_BEND_FOOT = 40.0      # foot-end Bezier handle length, mm
FOOT_D = 22.0              # domed foot: Ø22 cylinder + hemisphere, printed dome-up (was a Ø20 sphere)
FOOT_SCREW = "M3"
# ---------------------------------------------------------------- snowshoe foot (replaces the ball, design review v2)
# One printed part on the shank's foot plug: closed 0.8 mm skin + generative Voronoi ribs on top, rim, two cheeks
# and two lips. The existing M3 x 25 cross-bolt is the ankle (pitch) pivot; a wave washer under the nylock
# makes it a friction hinge so the pad holds its angle in swing. Lips stop the shank plug at ANKLE_LIM.
SHOE_L = 110.0              # teardrop planform: toe circle R_TOE + heel circle R_TAIL
SHOE_R_TOE = 35.0
SHOE_R_TAIL = 27.0
SHOE_PIVOT_AHEAD = 0.08     # pivot ahead of the planform centroid, fraction of SHOE_L
SHOE_PIVOT_UP = 15.0        # ankle pivot above the sole
SHOE_SKIN = 0.8             # closed skin on the snow side (4 layers)
SHOE_RIB_W = 1.2            # Voronoi ribs: 3 lines
SHOE_RIB_H = 3.0
SHOE_RIM_W = 2.4
SHOE_RIM_H = 6.0
SHOE_TOE_LIFT = 5.0         # toe edge chamfered up 5 mm ...
SHOE_TOE_RUN = 5.0          # ... over 5 mm (45 deg: a shallower bevel is an unsupported ceiling when printed sole-down)
SHOE_CELL = (5.5, 11.0)     # Voronoi seed spacing at the pivot -> at the far edge (generative grading)
SHOE_CHEEK_T = 4.0
SHOE_KEEL_W = 2.4           # three tapered keels (centre + two at +-SHOE_KEEL_Z): 10 mm tall at the hub, 6 at the ends;
SHOE_KEEL_H = 10.0          # without them the toe fails the rock-under-the-toe case (36 MPa, Kt 2)
SHOE_KEEL_Z = 18.0
SHOE_KEEL_HUB = 9.0         # the centre keel stops this far from the pivot (the plug end sweeps r 7.8 below it)
ANKLE_UP = 10.0             # pivot = the plug's M3 cross hole, this far up the foot axis from FOOT
ANKLE_LIM = (11.0, 52.0)    # shank angle in the pad frame at the lips (walking stance 16.1-47.3 deg, +-5)
ANKLE_BUILD = 32.0          # pad modelled at mid-stance (shank 32 deg from the pad normal)
ANKLE_LIP_H = 12.0          # lip contact height above the pivot (below the shank tip arc)
# effective lower link for IK / gait: knee -> ankle pivot
_t = math.radians(SHANK_FOOT_TILT_DEG)
L2_ANKLE = math.hypot(ANKLE_UP * math.sin(_t), L2 - ANKLE_UP * math.cos(_t))
ANKLE_OFF_DEG = math.degrees(math.atan2(-ANKLE_UP * math.sin(_t), L2 - ANKLE_UP * math.cos(_t)))

# ---------------------------------------------------------------- module frame
BRIDGE_Y = (MOTOR_D / 2 + 2.0, MOTOR_D / 2 + 8.0)   # top bridge, sets hip range
BRIDGE_W = 20.0
FLANGE_X = MOTOR_D / 2 + 4.0                        # module back/front flange inner face |x| (screw heads fit)
STUB_D = 5.0

# ---------------------------------------------------------------- body bracket
BB_BACK_R = 27.0            # back-wall disc radius about ab/ad axis (< thigh distance)
BB_FRONT_R = 22.0
SPINE_Z = (-54.0, -46.0)    # 8 mm: holds M3 heat-set inserts for the wall feet    # CAD z (ab/ad axis = 0): inboard spine, outside module sweep
RIG_SLOT_GRID = 20.0

# ---------------------------------------------------------------- joint limits (deg) â€” from CAD clearance
Q0_LIM = (-45.0, 45.0)
Q1_LIM = (-100.0, 100.0)
Q2_LIM = (15.0, 145.0)     # 145 with the snowshoe (it reaches the hip module at 150 when the hip is past +49 deg)

# ---------------------------------------------------------------- skeleton (section 5.1)
SK_FLANGE = 4.5
SK_STRUT = 4.0
SK_FILLET = 2.0
SK_THIGH_HOLES = 4
SK_SHANK_HOLES = 5
SK_MIN_HOLE = 6.0
SK_WEB = 3.0                # shank web left on the bed side under the pocket
SK_KNEE_SOLID_R = 13.0      # solid ring around the knee axle / hex pocket
# large rounded-rectangle cutouts (design review option B): 3 per link down the middle
SK_CUTS = 3
SK_CUT_WALL_THIGH = 5.0     # wall between neighbouring thigh cutouts
SK_CUT_WALL_SHANK = 7.0     # wall between shank cutouts (kept smaller than the thigh's for strength)
SK_CUT_INSET_SHANK = 2.5    # shank cutout edge inside the web-pocket edge (web ribs left along the flanges)
SK_CUT_R = 3.0              # cutout corner radius (shank 2.5)

# ---------------------------------------------------------------- material / print
PLA_STRENGTH = 25.0         # MPa in-plane effective
PLA_E = 2300.0              # MPa
PLA_RHO = 1.24e-6           # kg/mm^3 solid
SF = 2.0
BED = (220, 220, 250)


def as_dict():
    g = globals()
    return {k: g[k] for k in sorted(g) if k.isupper() and not callable(g[k])}


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "params.json"), "w") as f:
        json.dump(as_dict(), f, indent=1)
    for k in ("L1", "L2", "N_RATIO", "PD_HIP", "PD_KNEE", "L_ABAD", "Z_THIGH_IN", "Z_PLATE_IN", "Z_PLATE_OUT"):
        print(k, as_dict()[k])

