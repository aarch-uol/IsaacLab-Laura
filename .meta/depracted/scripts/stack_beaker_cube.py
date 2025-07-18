# #region Header
# IsaacLab Task Script
# --------------------------------------------------------------------------------
# File: stack_beaker_cube.py
# Author: Harry WF Cheung (University of Liverpool)
# Date: 16/06/2025
# Description: 
# --------------------------------------------------------------------------------
# License: BSD-3-Clause (© 2022-2025 Isaac Lab Project Developers)
#endregion

# --------------------------------------------------------------------------------
# region Imports

from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.manipulation.stack import mdp
from isaaclab.envs.mimic_env_cfg import MimicEnvCfg 
from isaaclab_tasks.manager_based.manipulation.stack.mdp import franka_stack_events
from isaaclab_tasks.manager_based.manipulation.stack.config.franka.stack_ik_rel_env_cfg import FrankaCubeStackEnvCfg
from isaaclab.managers import (
    ObservationGroupCfg as ObsGroup, 
    ObservationTermCfg as ObsTerm, 
    TerminationTermCfg as DoneTerm, 
    EventTermCfg as EventTerm, 
    SceneEntityCfg
)
from chills.envs.franka_env import  BaseSubTaskCfg
from chills.envs.labware_cfg import Beaker, RingStand
from chills.envs import mdp

import os 
cwd = os.getcwd()

@configclass
class ChillsStubtaskCfg(ObsGroup):
    """ Override Exisitng SubtaskCfg for beaker stack"""

    grasp_1 = ObsTerm(
        func=mdp.object_grasped,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "ee_frame_cfg": SceneEntityCfg("ee_frame"),
            "object_cfg": SceneEntityCfg("cube_2"),
        },
    )

    def __post_init__(self):
        self.enable_corruption = False
        self.concatenate_terms = False
    

@configclass
class ChillsTermCfg:
    """ Override Exisitng Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    cube_1_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("cube_1")}
    )

    cube_2_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("cube_2")}
    )

    success = DoneTerm(func=mdp.object_stacked,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "upper_object_cfg": SceneEntityCfg("cube_2"),
                "lower_object_cfg": SceneEntityCfg("cube_1"),
            },
        )

@configclass
class EventCfg:
    """ Modified events """

    randomize_cube_positions = EventTerm(
        func=franka_stack_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.4, 0.6), "y": (-0.10, 0.10), "z": (0.0203, 0.0203), "yaw": (-1.0, 1, 0)},
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("cube_1")],
        },
    )

    randomize_object_positions = EventTerm(
        func=franka_stack_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.4, 0.6), "y": (-0.20, 0.20), "z": (0.0503, 0.0503), "roll": (90.0, 90.0), "pitch": (0, 0),}, # adjust roll for startiong beaker orientation 
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("cube_2")],
        },
    )

@configclass
class StackBeaker(FrankaCubeStackEnvCfg, MimicEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.observations.SubtaskCfg = ChillsStubtaskCfg()
        self.observations.subtask_terms = ChillsStubtaskCfg()
        self.terminations = ChillsTermCfg()
        self.events = EventCfg()

        self.scene.cube_2 = Beaker.replace(prim_path="{ENV_REGEX_NS}/Beaker")
        self.scene.cube_1 = RingStand.replace(prim_path="{ENV_REGEX_NS}/RingStand")
        self.scene.cube_3.init_state.pos = ([2.0, -2, 0])

        self.datagen_config.name = "demo_src_stack_isaac_lab_task_D0"
        self.datagen_config.generation_keep_failed = True
        self.datagen_config.generation_num_trials = 10
        self.datagen_config.generation_select_src_per_subtask = True
        self.datagen_config.generation_relative = True
        self.datagen_config.max_num_failures = 25

        self.subtask_configs["franka"] =  {       
            BaseSubTaskCfg(
                object_ref="cube_2",
                subtask_term_signal="grasp_1",
                description="Grasp red cube",
                next_subtask_description="Stack red cube on top of blue cube",
            ),
            BaseSubTaskCfg(
                object_ref="cube_1",
                # End Term has Signal = None
                subtask_term_signal=None, 
                description="Place red cube on blue cube",
            )
        }