<<<<<<< HEAD
<<<<<<< HEAD
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e (Fresh start, no history)
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Velocity-Flat-Anymal-B-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:AnymalBFlatEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:AnymalBFlatPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_flat_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Velocity-Flat-Anymal-B-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.flat_env_cfg:AnymalBFlatEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:AnymalBFlatPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_flat_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Velocity-Rough-Anymal-B-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:AnymalBRoughEnvCfg",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:AnymalBRoughPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Velocity-Rough-Anymal-B-Play-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.rough_env_cfg:AnymalBRoughEnvCfg_PLAY",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:AnymalBRoughPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_rough_ppo_cfg.yaml",
    },
)
