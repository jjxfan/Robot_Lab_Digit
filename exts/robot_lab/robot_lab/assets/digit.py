"""Configuration for Unitree robots.

The following configurations are available:

* :obj:`UNITREE_A1_CFG`: Unitree A1 robot with DC motor model for the legs
* :obj:`G1_CFG`: G1 humanoid robot

Reference: https://github.com/unitreerobotics/unitree_ros
"""

from robot_lab.assets import ISAACLAB_ASSETS_DATA_DIR

import omni.isaac.lab.sim as sim_utils
from omni.isaac.lab.actuators import DCMotorCfg
from omni.isaac.lab.actuators import ImplicitActuatorCfg
from omni.isaac.lab.assets.articulation import ArticulationCfg

##
# Configuration
##


DIGIT_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_ASSETS_DATA_DIR}/Robots/Agility/digit-v3-clean.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.2),
        joint_pos={
            # '.*_leg_hip_roll': 0.0,
            # '.*_arm_shoulder_roll': 0.0,
            # '.*_leg_hip_yaw': 0.0,
            # '.*_arm_shoulder_pitch': 0.0,
            # '.*_leg_hip_pitch': 0.0,
            # '.*_arm_shoulder_yaw':0.0, 
            # '.*_leg_knee': 0.0, 
            # '.*_arm_elbow': 0.0, 
            # '.*_leg_shin': 0.0,
            # '.*_leg_tarsus': 0.0,
            # '.*_leg_heel_spring': 0.0,
            # '.*_leg_toe_a': 0.0,
            # '.*_leg_toe_b': 0.0, 
            # '.*_leg_toe_pitch' :0.0, 
            # '.*_leg_toe_roll': 0.0, 
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=[
            # '.*_leg_hip_roll',
            # '.*_arm_shoulder_roll',
            # '.*_leg_hip_yaw',
            # '.*_arm_shoulder_pitch',
            # '.*_leg_hip_pitch',
            # '.*_arm_shoulder_yaw', 
            # '.*_leg_knee', 
            # '.*_arm_elbow', 
            # '.*_leg_shin',
            # '.*_leg_tarsus',
            # '.*_leg_heel_spring',
            # '.*_leg_toe_a',
            # '.*_leg_toe_b', 
            # '.*_leg_toe_pitch', 
            # '.*_leg_toe_roll'
            ],
            effort_limit=200.0,
            velocity_limit=10.0,
            friction=0.0,
        ),
    },
)

