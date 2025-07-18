from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.manipulation.cube_lift.config.franka.dev_ik_rel_env_cfg import FrankaDevEnvCfg


@configclass
class CubeMimicEnvCfg(FrankaDevEnvCfg, MimicEnvCfg):
    def __post_init__(self):
        super().__post_init__()

        self.datagen_config.name = "beaker_rand_scale_D0"
        self.datagen_config.generation_guarantee = True
        self.datagen_config.generation_keep_failed = True
        self.datagen_config.generation_num_trials = 10
        self.datagen_config.generation_select_src_per_subtask = True
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True
        self.datagen_config.max_num_failures = 25
        self.datagen_config.seed = 1

        subtask_configs = []
        subtask_configs.append(
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="appr",
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.03,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )
        subtask_configs.append(
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="grasp",
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.03,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )
        subtask_configs.append(
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="lift",
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_robot_distance",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.03,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )
        subtask_configs.append(
            SubTaskConfig(
                object_ref="object",
                subtask_term_signal="appr_goal",
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_robot_distance",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.03,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )
        self.subtask_configs["franka"] = subtask_configs
