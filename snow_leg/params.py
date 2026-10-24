"""SNOW LEG — single source of truth for the Arctic search-and-rescue variant.
Copied from the Leg Day params.py (root folder, untouched) and extended with the
SNOW, INSERTS and SNOW_MODE blocks. All lengths in mm, angles in degrees.

Body/CAD frame (brief section 4.1): origin = hip-pitch axis ∩ ab/ad axis.
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
FILE_PREFIX = "snow_"       # every SolidWorks file of this variant (SolidWorks needs unique names)
FOOT = os.environ.get("SNOW_FOOT", "snowshoe")          # "snowshoe" | "ball"
SNOW_MODE = os.environ.get("SNOW_MODE", "1") == "1"     # snow stance + gait (analysis/gait_check.py)
THIGH_HOLES = False         # Leg Day truss holes opened into the belt channel; closed for snow (saved only 3 %)
SHANK_CLOSED = True         # closed, smooth shank (no web pocket / truss hole that packs with snow)
FOOT_WINGS = False          # stretch goal (brief 3.7): not built
SKELETON = os.environ.get("LEG_SKELETON", "1") == "1"   # measurement switch: LEG_SKELETON=0 builds solid variants only
MOTOR_OUTPUT_FACE = "right_inner"   # Ø27 3xM2.5 + Ø15 boss = rotor (ASSUMED, verify by hand)

# ---------------------------------------------------------------- motor AK40-10 (sheet)
MOTOR_D = 53.0
MOTOR_L = 37.0
MOTOR_MASS = 0.185          # kg (sheet; catalog 0.200)
TAU_RATED = 1.3             # Nm at output
TAU_PEAK = 4.1
SPEED_RATED = 370.0         # rpm
KT_OUT = 0.56               # Nm/A at output
BACKLASH_ARCMIN = 18.0
ROTOR_INERTIA = 540e-7      # kg m^2 (catalog 540 g cm^2 — VERIFY, may be pre-gearbox)
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
FIT_LOCATE = 0.20           # Ø15 boss pocket etc.
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
THIGH_R_HIP = 24.0          # outer radius at hip end (covers Ø27 PCD + heads)
THIGH_R_KNEE = PD_KNEE / 2 + 7.0
THIGH_CH_R_HIP = 17.5       # channel inner radius at hip (screw heads inside)
THIGH_CH_R_KNEE = PD_KNEE / 2 + 3.0
IDLER_BOLT = "M3"
IDLER_Y = -40.0             # idler slot centre (below hip)
IDLER_SLOT_TRAVEL = 9.0     # >= 8 mm

# ---------------------------------------------------------------- shank + foot
SHANK_W_KNEE = 26.0
SHANK_W_FOOT = 14.0
FOOT_D = 20.0
FOOT_SCREW = "M3"

# ---------------------------------------------------------------- module frame
BRIDGE_Y = (MOTOR_D / 2 + 2.0, MOTOR_D / 2 + 8.0)   # top bridge, sets hip range
BRIDGE_W = 20.0
FLANGE_X = MOTOR_D / 2 + 4.0                        # module back/front flange inner face |x| (screw heads fit)
STUB_D = 5.0

# ---------------------------------------------------------------- body bracket
BB_BACK_R = 27.0            # back-wall disc radius about ab/ad axis (< thigh distance)
BB_FRONT_R = 22.0
SPINE_Z = (-52.0, -46.0)    # CAD z (ab/ad axis = 0): inboard spine, outside module sweep
RIG_SLOT_GRID = 20.0

# ---------------------------------------------------------------- joint limits (deg) — from CAD clearance
Q0_LIM = (-45.0, 45.0)
Q1_LIM = (-100.0, 100.0)
Q2_LIM = (15.0, 137.0)        # snow leg: the 120 x 80 foot reaches the hip pod beyond ~140 deg (clearance_check); gait needs 135

# ---------------------------------------------------------------- skeleton (section 5.1)
SK_FLANGE = 4.5
SK_STRUT = 4.0
SK_FILLET = 2.0
SK_THIGH_HOLES = 4
SK_SHANK_HOLES = 5
SK_MIN_HOLE = 6.0
SK_WEB = 3.0                # shank web left on the bed side under the pocket
SK_KNEE_SOLID_R = 13.0      # solid ring around the knee axle / hex pocket

# ================================================================ SNOW (brief SNOW_LEG_PROMPT.md)
ROBOT_MASS_SNOW = 4.5       # kg, ASSUMED (team default: 4.3 kg gait-study robot + snow feet)
G_ACC = 9.81
# --- snowshoe planform: teardrop = toe circle + tail circle joined by tangents (Top-plane slabs)
FOOT_L = 120.0              # overall length along X
FOOT_W = 80.0               # overall width along Z (= 2 x toe radius)
FOOT_R_TOE = FOOT_W / 2
FOOT_R_TAIL = 34.0          # slightly tapered tail (68 mm wide)
FOOT_C_DIST = FOOT_L - FOOT_R_TOE - FOOT_R_TAIL      # toe-centre to tail-centre distance
DECK_T = 2.0                # deck (brief 2-3 mm); stiffened by the lugs + rim
LUG_H = 5.0                 # transverse lugs + perimeter rim (brief 4-6 mm)
LUG_W = 1.6                 # 4 perimeters of 0.4 mm
RIM_W = 1.6
N_LUGS = 5
PAP_D = 1.0                 # papillae (Orndorf et al. 2022 scale), coupon-tested
PAP_H = 1.2
PAP_PITCH = 2.5
PAP_MAX = 300               # cap on papillae count (rebuild time)
PAD_PROUD = 0.0             # stepped 'hoof' pad below the lug tips; 0/6/12 evaluated in snow_model -> 0 chosen
PAD_L = 44.0                # central pad under the ankle (also carries the bracket inserts)
PAD_W = 34.0
TOE_BEVEL_RUN = 10.0        # sole-side 45 deg toe bevel
SPIKE_N = 4                 # M3 set-screw ice spikes in M3 inserts, forefoot + heel
SPIKE_PROTRUDE = 3.0        # past the lug tips (set screw sunk 2 mm below the top face)
# --- passive ankle
PIVOT_AHEAD_FRAC = 0.10     # pivot ahead of the planform centroid, fraction of FOOT_L (brief 10-20 %)
PIVOT_ABOVE_DECK = 10.0     # pivot axis above the deck top face (shank tip R9 + 1 mm)
ANKLE_PIN_D = 5.0           # M5 bolt shank, clamped by the cheeks
ANKLE_BUSH_FIT = 0.30       # shank-tip bushing on the pin (brief: 625-2RS or +0.3 bushing)
SHANK_TIP_R = 9.0
CHEEK_T = 4.0               # ankle bracket cheeks (two mirror parts, printed flat -> pivot holes vertical)
STOP_R = 20.0              # cross-bolt stops (M3 + Ø5 sleeve) radius from the pivot
STOP_SLEEVE_D = 5.0
SPRING_K = 0.10             # N/mm, small steel extension spring (ASSUMED stock)
SPRING_PRELOAD = 2.0        # N at the toe-up stop
ANKLE_MARGIN = 15.0         # deg added each side of the stance shank-angle range (brief 3.3)
FOOT_MASS_BUDGET = 35.0     # g
# --- snow stance + gait (SNOW_MODE)
SNOW_STANCE_FRAC = 0.92     # ankle-pivot height / (L1+L2); taller "stilts" stance (P4) -> knee bend 46 deg
SNOW_STEP = 0.120           # m stride
SNOW_PERIOD = 1.6           # s, slower in snow (P6)
SNOW_DUTY = 0.75            # walk, 3 feet down
SNOW_LIFT_FRAC = 0.25       # fraction of swing spent lifting vertically before moving forward
SNOW_SWING_MARGIN = 30.0    # mm above predicted sinkage (brief 4.4)
BODY_BELOW_HIP = 40.0       # mm, belly below the hip axis (ASSUMED body box)
HIP_SPACING_SNOW = 0.32     # m fore-aft; with stride/duty = 0.16 m -> direct register every 2 strides

# ================================================================ INSERTS (heat-set, brass)
INSERTS = {
    # size: hole Ø as modelled (+0.1 print compensation), insert length, design capacity (N)
    "M3": dict(hole=4.1, length=5.7, od=4.6, cap=800.0),      # brief 6.3 design value; CNC Kitchen ~1.78 kN test
    "M5": dict(hole=6.5, length=9.5, od=7.0, cap=2000.0),     # ASSUMED: M3 value x insert surface-area ratio, coupon-test
}
INSERT_PITCH = {"M3": 0.5, "M5": 0.8}
INSERT_WALL = 3.0           # min wall around an insert hole (JLC3DP)
INSERT_SF = 3.0
FALL_G = 3.0                # fall / stumble load = 3 x robot weight on one foot (ASSUMED)


def insert_depth(size):
    return INSERTS[size]["length"] + 2 * INSERT_PITCH[size]


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

