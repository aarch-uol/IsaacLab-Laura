# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: Apache-2.0

from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.utils import configclass

from isaaclab_tasks.manager_based.manipulation.stack.config.franka.LLM_ik_rel_env_cfg import FrankaCubeStackEnvCfg as generated_llm


@configclass
class LLMIKAbsMimicEnvCfg(generated_llm, MimicEnvCfg):
    """
    Isaac Lab Mimic environment config class for Franka Cube Stack IK Abs env.
    """

    def __post_init__(self):
        # post init of parents
        super().__post_init__()

        # Override the existing values
        self.datagen_config.name = "demo_src_stack_isaac_lab_task_D0"
        self.datagen_config.generation_guarantee = True
        self.datagen_config.generation_keep_failed = True
        self.datagen_config.generation_num_trials = 10
        self.datagen_config.generation_select_src_per_subtask = True
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True
        self.datagen_config.max_num_failures = 25
        self.datagen_config.seed = 1

        # The following are the subtask configurations for the stack task.

        # remove these as needed
        subtask_configs = []
        subtask_configs.append(
            SubTaskConfig(
                object_ref="object1",
                subtask_term_signal="reach_object_task1",
                subtask_term_offset_range=(10, 20),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.03,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )
        
        # subtask_configs.append(
        #     SubTaskConfig(
        #         object_ref="object_1",
        #         subtask_term_signal="is_object_lifted_task1",
        #         subtask_term_offset_range=(10, 20),
        #         selection_strategy="nearest_neighbor_object",
        #         selection_strategy_kwargs={"nn_k": 3},
        #         action_noise=0.01,
        #         num_interpolation_steps=5,
        #         num_fixed_steps=0,
        #         apply_noise_during_interpolation=False,
        #     )
        # )
        # subtask_configs.append(
        #     SubTaskConfig(
        #         object_ref="object_2",
        #         subtask_term_signal="object_reached_midgoal_task1",
        #         subtask_term_offset_range=(10, 20),
        #         selection_strategy="nearest_neighbor_object",
        #         selection_strategy_kwargs={"nn_k": 3},
        #         action_noise=0.01,
        #         num_interpolation_steps=5,
        #         num_fixed_steps=0,
        #         apply_noise_during_interpolation=False,
        #     )
        # )
        # subtask_configs.append(
        #     SubTaskConfig(
        #         object_ref="object_1",
        #         subtask_term_signal="reach_object2_task1",
        #         subtask_term_offset_range=(10, 20),
        #         selection_strategy="nearest_neighbor_object",
        #         selection_strategy_kwargs={"nn_k": 3},
        #         action_noise=0.01,
        #         num_interpolation_steps=5,
        #         num_fixed_steps=0,
        #         apply_noise_during_interpolation=False,
        #     )
        # )
        subtask_configs.append(
            SubTaskConfig(
                object_ref="object1",
                subtask_term_signal="object_stacked_task1",
                subtask_term_offset_range=(10, 20),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.01,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )
        # subtask_configs.append(
        #     SubTaskConfig(
        #         object_ref="object_1",
        #         subtask_term_signal="reach_object_task2",
        #         subtask_term_offset_range=(10, 20),
        #         selection_strategy="nearest_neighbor_object",
        #         selection_strategy_kwargs={"nn_k": 3},
        #         action_noise=0.01,
        #         num_interpolation_steps=5,
        #         num_fixed_steps=0,
        #         apply_noise_during_interpolation=False,
        #     )
        # )
        # subtask_configs.append(
        #     SubTaskConfig(
        #         object_ref="object_1",
        #         subtask_term_signal="object_grasped_task2",
        #         subtask_term_offset_range=(10, 20),
        #         selection_strategy="nearest_neighbor_object",
        #         selection_strategy_kwargs={"nn_k": 3},
        #         action_noise=0.01,
        #         num_interpolation_steps=5,
        #         num_fixed_steps=0,
        #         apply_noise_during_interpolation=False,
        #     )
        # )
        # subtask_configs.append(
        #     SubTaskConfig(
        #         object_ref="object_1",
        #         subtask_term_signal="is_object_lifted_task2",
        #         subtask_term_offset_range=(10, 20),
        #         selection_strategy="nearest_neighbor_object",
        #         selection_strategy_kwargs={"nn_k": 3},
        #         action_noise=0.01,
        #         num_interpolation_steps=5,
        #         num_fixed_steps=0,
        #         apply_noise_during_interpolation=False,
        #     )
        # )
        # subtask_configs.append(
        #     SubTaskConfig(
        #         object_ref="object_1",
        #         subtask_term_signal="object_reached_midgoal_task2",
        #         subtask_term_offset_range=(10, 20),
        #         selection_strategy="nearest_neighbor_object",
        #         selection_strategy_kwargs={"nn_k": 3},
        #         action_noise=0.01,
        #         num_interpolation_steps=5,
        #         num_fixed_steps=0,
        #         apply_noise_during_interpolation=False,
        #     )
        # )
        # subtask_configs.append(
        #     SubTaskConfig(
        #         object_ref="object_1",
        #         subtask_term_signal="reach_object2_task2",
        #         subtask_term_offset_range=(10, 20),
        #         selection_strategy="nearest_neighbor_object",
        #         selection_strategy_kwargs={"nn_k": 3},
        #         action_noise=0.01,
        #         num_interpolation_steps=5,
        #         num_fixed_steps=0,
        #         apply_noise_during_interpolation=False,
        #     )
        # )
        subtask_configs.append(
            SubTaskConfig(
                object_ref="object1",
                subtask_term_signal="object_stacked_task2",
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.01,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )
        subtask_configs.append(
            SubTaskConfig(
                object_ref="object1",
                subtask_term_signal=None,
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object",
                selection_strategy_kwargs={"nn_k": 3},
                action_noise=0.01,
                num_interpolation_steps=5,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
            )
        )
        self.subtask_configs["franka"] = subtask_configs
