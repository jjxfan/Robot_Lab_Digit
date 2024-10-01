from robot_lab.tasks.locomotion.velocity.velocity_env_cfg import LocomotionVelocityRoughEnvCfg

from omni.isaac.lab.utils import configclass

##
# Pre-defined configs
##
# use cloud assets
# from omni.isaac.lab_assets.unitree import UNITREE_A1_CFG  # isort: skip
# use local assets
from robot_lab.assets.digit import DIGIT_CFG  # isort: skip


@configclass
class DigitRoughEnvCfg(LocomotionVelocityRoughEnvCfg):
    _run_disable_zero_weight_rewards = True

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # ------------------------------Sence------------------------------
        # switch robot to unitree-a1
        self.scene.robot = DIGIT_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.height_scanner.prim_path = "{ENV_REGEX_NS}/Robot/torso_base"
        # scale down the terrains because the robot is small
        self.scene.terrain.terrain_generator.sub_terrains["boxes"].grid_height_range = (0.025, 0.1)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_range = (0.01, 0.06)
        self.scene.terrain.terrain_generator.sub_terrains["random_rough"].noise_step = 0.01

        # ------------------------------Observations------------------------------
        self.observations.policy.base_lin_vel.scale = 2.0
        self.observations.policy.base_ang_vel.scale = 0.25
        self.observations.policy.joint_pos.scale = 1.0
        self.observations.policy.joint_vel.scale = 0.05
        self.observations.policy.base_lin_vel = None
        self.observations.policy.height_scan = None
        self.observations.AMP = None

        # ------------------------------Actions------------------------------
        # reduce action scale
        self.actions.joint_pos.scale = 0.25
        self.actions.joint_pos.clip = {".*": (-100, 100)}

        # ------------------------------Events------------------------------
        self.events.reset_amp = None
        self.events.add_base_mass.params["mass_distribution_params"] = (-1.0, 3.0)
        self.events.add_base_mass.params["asset_cfg"].body_names = ["torso_base"]
        self.events.base_external_force_torque.params["asset_cfg"].body_names = ["torso_base"]
        self.events.reset_base.params = {
            "pose_range": {"x": (-0.5, 0.5), "y": (-0.5, 0.5), "yaw": (-3.14, 3.14)},
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-0.5, 0.5),
            },
        }
        self.events.randomize_actuator_gains = None
        self.events.randomize_joint_parameters = None

        # ------------------------------Rewards------------------------------
        # General
        # UNUESD self.rewards.is_alive.weight = 0
        self.rewards.is_terminated.weight = 0

        # Root penalties
        self.rewards.lin_vel_z_l2.weight = -2.0
        self.rewards.ang_vel_xy_l2.weight = -0.05
        self.rewards.flat_orientation_l2.weight = -0.5
        self.rewards.base_height_l2.weight = 0
        self.rewards.base_height_l2.params["target_height"] = 0.35
        self.rewards.base_height_l2.params["asset_cfg"].body_names = ["torso_base"]
        self.rewards.body_lin_acc_l2.weight = 0
        self.rewards.body_lin_acc_l2.params["asset_cfg"].body_names = ["torso_base"]

        # Joint penaltie
        self.rewards.joint_torques_l2.weight = -0.0002
        # UNUESD self.rewards.joint_vel_l1.weight = 0.0
        self.rewards.joint_vel_l2.weight = 0
        self.rewards.joint_acc_l2.weight = -2.5e-7
        # self.rewards.create_joint_deviation_l1_rewterm("joint_deviation_l1", 0, [""])
        self.rewards.joint_pos_limits.weight = -5.0
        self.rewards.joint_vel_limits.weight = 0

        # Action penalties
        self.rewards.applied_torque_limits.weight = 0
        self.rewards.applied_torque_limits.params["asset_cfg"].body_names = ["torso_base"]
        self.rewards.action_rate_l2.weight = -0.01
        # UNUESD self.rewards.action_l2.weight = 0.0

        # Contact sensor
        self.rewards.undesired_contacts.weight = -1.0
        self.rewards.undesired_contacts.params["sensor_cfg"].body_names = [
            'torso_base', 'left_leg_hip_roll', 'left_leg_hip_yaw', 'left_leg_hip_pitch', 'left_leg_achilles_rod', 'left_leg_knee', 'left_leg_shin', 'left_leg_tarsus', 'left_leg_heel_spring', 'left_leg_toe_a', 'left_leg_toe_a_rod', 'left_leg_toe_b', 'left_leg_toe_b_rod', 'left_arm_shoulder_roll', 'left_arm_shoulder_pitch', 'left_arm_shoulder_yaw', 'left_arm_elbow', 'right_leg_hip_roll', 'right_leg_hip_yaw', 'right_leg_hip_pitch', 'right_leg_achilles_rod', 'right_leg_knee', 'right_leg_shin', 'right_leg_tarsus', 'right_leg_heel_spring', 'right_leg_toe_a', 'right_leg_toe_a_rod', 'right_leg_toe_b', 'right_leg_toe_b_rod', 'right_arm_shoulder_roll', 'right_arm_shoulder_pitch', 'right_arm_shoulder_yaw', 'right_arm_elbow']
        self.rewards.contact_forces.weight = 0
        self.rewards.contact_forces.params["sensor_cfg"].body_names = [".*_leg_toe_roll"]

        # Velocity-tracking rewards
        self.rewards.track_lin_vel_xy_exp.weight = 1.5
        self.rewards.track_ang_vel_z_exp.weight = 0.7

        # Others
        self.rewards.feet_air_time.weight = 0.01
        self.rewards.feet_air_time.params["sensor_cfg"].body_names = [".*_leg_toe_roll"]
        self.rewards.foot_contact.weight = 0
        self.rewards.foot_contact.params["sensor_cfg"].body_names = [".*_leg_toe_roll"]
        self.rewards.base_height_rough_l2.weight = 0
        self.rewards.base_height_rough_l2.params["target_height"] = 0.35
        self.rewards.base_height_rough_l2.params["asset_cfg"].body_names = ["torso_base"]
        self.rewards.feet_slide.weight = 0
        self.rewards.feet_slide.params["sensor_cfg"].body_names = [".*_leg_toe_roll"]
        self.rewards.feet_slide.params["asset_cfg"].body_names = [".*_leg_toe_roll"]
        self.rewards.joint_power.weight = -2e-5
        self.rewards.stand_still_when_zero_command.weight = -0.5

        # If the weight of rewards is 0, set rewards to None
        if self._run_disable_zero_weight_rewards:
            self.disable_zero_weight_rewards()

        # ------------------------------Terminations------------------------------
        self.terminations.illegal_contact.params["sensor_cfg"].body_names = ['torso_base']

        # ------------------------------Commands------------------------------
