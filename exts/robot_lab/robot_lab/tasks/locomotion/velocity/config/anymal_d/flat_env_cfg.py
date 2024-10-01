from omni.isaac.lab.utils import configclass

from .rough_env_cfg import AnymalDRoughEnvCfg


@configclass
class AnymalDFlatEnvCfg(AnymalDRoughEnvCfg):
    def __post_init__(self):
        # Temporarily not run disable_zerow_eight_rewards() in parent class to override rewards
        self._run_disable_zero_weight_rewards = False
        # post init of parent
        super().__post_init__()

        # override rewards
        self.rewards.flat_orientation_l2.weight = -5.0
        self.rewards.joint_torques_l2.weight = -2.5e-5
        self.rewards.feet_air_time.weight = 0.5
        self.rewards.base_height_l2.weight = 0
        self.rewards.base_height_rough_l2.weight = 0
        # change terrain to flat
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        # no height scan
        self.scene.height_scanner = None
        self.observations.policy.height_scan = None
        # no terrain curriculum
        self.curriculum.terrain_levels = None

        # Now executing disable_zerow_eight_rewards()
        self._run_disable_zero_weight_rewards = True
        if self._run_disable_zero_weight_rewards:
            self.disable_zero_weight_rewards()
