#region Header
# IsaacLab Task Script
# --------------------------------------------------------------------------------
# File: stack_obj_on_base
# Author: Harry WF Cheung (University of Liverpool)
# Date: 16/06/2025
# Description: 
# --------------------------------------------------------------------------------
# License: BSD-3-Clause (© 2022-2025 Isaac Lab Project Developers)
#endregion


# --------------------------------------------------------------------------------
# region Imports

from isaaclab.utils import configclass
from isaaclab.managers import (
    ObservationGroupCfg as ObsGroup,
    ObservationTermCfg as ObsTerm,
    TerminationTermCfg as DoneTerm,
    EventTermCfg as EventTerm,
    SceneEntityCfg,
)
from chills.envs import mdp, Beaker, RingStand, FrankaIkEnvCfg

@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""
    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group with state values."""
        actions = ObsTerm(func=mdp.last_action)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object = ObsTerm(func=mdp.object_obs)
        obj_positions = ObsTerm(func=mdp.obj_in_world_frame)
        obj_orientations = ObsTerm(func=mdp.obj_orientations_in_world)
        eef_pos = ObsTerm(func=mdp.ee_frame_pos)
        eef_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class SubtaskCfg(ObsGroup):
        """Observations for subtask group."""
        grasp_1 = ObsTerm( func=mdp.object_grasped, params={
            "robot_cfg": SceneEntityCfg("robot"), "ee_frame_cfg": SceneEntityCfg("ee_frame"), "object_cfg": SceneEntityCfg("obj_2"),
            }, )
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    subtask_terms: SubtaskCfg = SubtaskCfg()

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    obj_1_dropping = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("obj_1")})
    obj_2_dropping = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("obj_2")})
    success = DoneTerm(func=mdp.object_stacked, params={
                "robot_cfg": SceneEntityCfg("robot"), "upper_object_cfg": SceneEntityCfg("obj_2"), "lower_object_cfg": SceneEntityCfg("obj_1"),
        },)

@configclass
class EventCfg:
    """Configuration for events."""
    init_franka_arm_pose = EventTerm(
        func=mdp.set_default_joint_pose, mode="startup", params={
            "default_pose": [0.0444, -0.1894, -0.1107, -2.5148, 0.0044, 2.3775, 0.6952, 0.0400, 0.0400],
    },)
    randomize_franka_joint_state = EventTerm(
        func=mdp.randomize_joint_by_gaussian_offset, mode="reset", params={
            "mean": 0.0, "std": 0.02, "asset_cfg": SceneEntityCfg("robot"),
    },)

@configclass
class StackObjOnBase(FrankaIkEnvCfg):
    """Configuration for the test environment inheriting TableSceneCfg anf IK Panda."""

    observations: ObservationsCfg = ObservationsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    command: object = None 
    rewards: object = None 
    curriculum: object = None 

    def __post_init__(self):
        super().__post_init__()
        self.scene.obj_2 = Beaker.replace(prim_path="{ENV_REGEX_NS}/Beaker")
        self.scene.obj_2.spawn.semantic_tags = [("class", "obj_2")]
        self.scene.obj_1 = RingStand.replace(prim_path="{ENV_REGEX_NS}/RingStand")
        self.scene.obj_2.spawn.semantic_tags = [("class", "obj_1")]

        
        