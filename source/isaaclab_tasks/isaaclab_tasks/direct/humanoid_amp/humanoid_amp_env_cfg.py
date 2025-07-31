<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e (Fresh start, no history)
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> e9462be776417c5794982ad017c44c19fac790a2
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import os
from dataclasses import MISSING

from isaaclab_assets import HUMANOID_28_CFG

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import PhysxCfg, SimulationCfg
from isaaclab.utils import configclass

MOTIONS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "motions")


@configclass
class HumanoidAmpEnvCfg(DirectRLEnvCfg):
    """Humanoid AMP environment config (base class)."""

    # env
    episode_length_s = 10.0
    decimation = 2

    # spaces
    observation_space = 81
    action_space = 28
    state_space = 0
    num_amp_observations = 2
    amp_observation_space = 81

    early_termination = True
    termination_height = 0.5

    motion_file: str = MISSING
    reference_body = "torso"
    reset_strategy = "random"  # default, random, random-start
    """Strategy to be followed when resetting each environment (humanoid's pose and joint states).

    * default: pose and joint states are set to the initial state of the asset.
    * random: pose and joint states are set by sampling motions at random, uniform times.
    * random-start: pose and joint states are set by sampling motion at the start (time zero).
    """

    # simulation
    sim: SimulationCfg = SimulationCfg(
        dt=1 / 60,
        render_interval=decimation,
        physx=PhysxCfg(
            gpu_found_lost_pairs_capacity=2**23,
            gpu_total_aggregate_pairs_capacity=2**23,
        ),
    )

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=10.0, replicate_physics=True)

    # robot
    robot: ArticulationCfg = HUMANOID_28_CFG.replace(prim_path="/World/envs/env_.*/Robot").replace(
        actuators={
            "body": ImplicitActuatorCfg(
                joint_names_expr=[".*"],
<<<<<<< HEAD
<<<<<<< HEAD
=======
                velocity_limit=100.0,
>>>>>>> abfba5273e (Fresh start, no history)
=======
                velocity_limit=100.0,
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
                stiffness=None,
                damping=None,
            ),
        },
    )


@configclass
class HumanoidAmpDanceEnvCfg(HumanoidAmpEnvCfg):
    motion_file = os.path.join(MOTIONS_DIR, "humanoid_dance.npz")


@configclass
class HumanoidAmpRunEnvCfg(HumanoidAmpEnvCfg):
    motion_file = os.path.join(MOTIONS_DIR, "humanoid_run.npz")


@configclass
class HumanoidAmpWalkEnvCfg(HumanoidAmpEnvCfg):
    motion_file = os.path.join(MOTIONS_DIR, "humanoid_walk.npz")
