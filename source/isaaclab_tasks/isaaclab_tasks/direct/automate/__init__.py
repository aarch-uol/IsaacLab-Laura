<<<<<<< HEAD
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-AutoMate-Assembly-Direct-v0",
    entry_point=f"{__name__}.assembly_env:AssemblyEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.assembly_env:AssemblyEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
    },
)


gym.register(
    id="Isaac-AutoMate-Disassembly-Direct-v0",
    entry_point=f"{__name__}.disassembly_env:DisassemblyEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.disassembly_env:DisassemblyEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
    },
)
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import gymnasium as gym

from . import agents
from .assembly_env import AssemblyEnv, AssemblyEnvCfg
from .disassembly_env import DisassemblyEnv, DisassemblyEnvCfg

##
# Register Gym environments.
##

gym.register(
    id="Isaac-AutoMate-Assembly-Direct-v0",
    entry_point="isaaclab_tasks.direct.automate:AssemblyEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": AssemblyEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
    },
)


gym.register(
    id="Isaac-AutoMate-Disassembly-Direct-v0",
    entry_point="isaaclab_tasks.direct.automate:DisassemblyEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": DisassemblyEnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
    },
)
>>>>>>> harry/main
