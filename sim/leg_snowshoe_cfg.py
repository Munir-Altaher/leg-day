"""Isaac Lab articulation config for the free-standing leg on its snowshoe (sim/leg_snowshoe.urdf).

Written against the Isaac Lab 2.x API (isaaclab.*). Class and field names move between releases:
if an import or field fails, check it against your installed version's docs.
"""
import math
import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg, ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg

HERE = os.path.dirname(os.path.abspath(__file__))

# AK40-10 at the output: 4.1 N m peak, 1.3 N m rated, 370 rpm rated (38.7 rad/s)
TAU_PEAK, TAU_RATED, W_MAX = 4.1, 1.3, 38.7
# knee: belt-limited (120 N peak x 19.1 mm pulley radius), motor speed / 3 through the 20T:60T belt
KNEE_TAU, KNEE_W = 2.29, 12.9

# standing pose. The ankle is a free friction hinge, so the leg can only stand still leaning on a lip: shank at the
# rear stop (52 deg in the pad frame) with the hip over the middle of the shoe. Checked in MuJoCo (sim/build_mujoco.py
# --check): stands on a hard floor; tips on soft contact, since the CoM is only ~12 mm inside the shoe's inboard edge.
ANKLE_STOP = math.radians(52.0)
Q1 = math.radians(-40.0)
Q2 = ANKLE_STOP - Q1                   # q1 + q2 = the stop -> shoe flat
ANKLE = -ANKLE_STOP
HIP_HEIGHT = 0.186 + 0.003             # hip above the floor with the sole down, + 3 mm drop-in margin

LEG_SNOWSHOE_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=os.path.join(HERE, "leg_snowshoe.urdf"),
        fix_base=False,                                  # free-standing on the snowshoe
        merge_fixed_joints=True,
        activate_contact_sensors=True,
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)),
        rigid_props=sim_utils.RigidBodyPropertiesCfg(max_depenetration_velocity=5.0),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=1),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, HIP_HEIGHT),
        rot=(0.70711, 0.70711, 0.0, 0.0),               # (w, x, y, z): +90 deg about X, CAD Y-up -> sim Z-up
        joint_pos={"abad": 0.0, "hip": Q1, "knee": Q2, "ankle": ANKLE},
    ),
    actuators={
        "abad_hip": DCMotorCfg(joint_names_expr=["abad", "hip"], saturation_effort=TAU_PEAK, effort_limit=TAU_PEAK,
                               velocity_limit=W_MAX, stiffness=20.0, damping=0.5),
        # NOTE: the knee motor sits at the hip and drives the knee through the belt (motor angle = q1 + 3 q2),
        # so its torque tau_m gives knee torque 3 tau_m AND adds tau_m to the hip joint. This config limits the
        # knee joint torque only; see README.md for the coupled model to use before sim-to-real.
        "knee": DCMotorCfg(joint_names_expr=["knee"], saturation_effort=KNEE_TAU, effort_limit=KNEE_TAU,
                           velocity_limit=KNEE_W, stiffness=20.0, damping=0.5),
        # passive snowshoe ankle: friction hinge (~0.09 N m), lips at 11 and 52 deg are the URDF joint limits
        "ankle": ImplicitActuatorCfg(joint_names_expr=["ankle"], stiffness=0.0, damping=0.002, friction=0.09,
                                     effort_limit=0.0),
    },
)
