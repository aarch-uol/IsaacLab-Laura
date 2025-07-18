#region Header
# IsaacLab Beaker Lift Task
# --------------------------------------------------------------------------------
# File: lift_labware.py
# Author: Harry WF Cheung (University of Liverpool)
# Date: 14/06/2025 - Updated 16/06/2025
# Description: Inherits Franka IK Rel Lift with custom labware
# --------------------------------------------------------------------------------
# License: BSD-3-Clause (© 2022-2025 Isaac Lab Project Developers)
#endregion 


# --------------------------------------------------------------------------------
# region Imports

from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.manipulation.lift import mdp
from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import (
     RewardsCfg, CurriculumCfg)
from isaaclab.managers import (
    ObservationGroupCfg as ObsGroup, 
    ObservationTermCfg as ObsTerm,
    TerminationTermCfg as DoneTerm,
    EventTermCfg as EventTerm,
    SceneEntityCfg)
from chills.envs import FrankaIkEnvCfg, mdp, Beaker, Hotplate, PARALLEL_IK_FRANKA


from isaaclab.assets import RigidObjectCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg


# --------------------------------------------------------------------------------
# region MDP: Observation

@configclass
class ObsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""
        # region Obs: Policies
        actions = ObsTerm(func=mdp.last_action)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
        eef_pos = ObsTerm(func=mdp.ee_frame_pos)
        eef_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)

        # Cube has but i don't 
        # object_to_target = ObsTerm(func=mdp.object_reached_goal)



        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False # Has to be False

    @configclass
    class SubtaskCfg(ObsGroup):
        """ Mirrored subtasks observation (Matches Mimic Subtasks)"""
        # region Obs: Subtasks
        appr = ObsTerm(func=mdp.reach_object, params={
            "ee_frame_cfg": SceneEntityCfg("ee_frame"), "object_cfg": SceneEntityCfg("object"), "std" : 0.05}) 
        grasp = ObsTerm( func=mdp.object_grasped, params={ 
            "robot_cfg": SceneEntityCfg("robot"), "ee_frame_cfg": SceneEntityCfg("ee_frame"), "object_cfg": SceneEntityCfg("object")})  
        lift = ObsTerm(func=mdp.is_object_lifted, params={
            "obj_cfg": SceneEntityCfg("object"), "threshold" : 0.05 })        
        appr_goal = ObsTerm(  func=mdp.object_near_goal, params={  
            "threshold": 0.05,  "command_name": "object_pose", "object_cfg": SceneEntityCfg("object"), })
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False # Has to be False

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    subtask_terms : SubtaskCfg = SubtaskCfg()


# --------------------------------------------------------------------------------
# region MDP: Events (resets) 

@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_object_position = EventTerm( func=mdp.reset_root_state_uniform, mode="reset", params={
            "pose_range": {"x": (-0.1, 0.2), "y": (-0.35, -0.1), "z": (0.0, 0.0)}, "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object", body_names="object"),
    } )

# --------------------------------------------------------------------------------
# region MDP: Command 

goal_x, goal_y, goal_z = 0.45, -0.25, 0.06

@configclass
class CommandsCfg:
    """ Defines the Goal to be a static location "goal" above the hotplat"""
    object_pose = mdp.UniformPoseCommandCfg(
        asset_name = "robot",
        body_name = "panda_hand",
        resampling_time_range = (1.0e9, 1.0e9),
        debug_vis = False,
        ranges = mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(goal_x, goal_x), pos_y=(goal_y, goal_y), pos_z=(goal_z, goal_z), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
    ))

# --------------------------------------------------------------------------------
# region MDP: Termination

@configclass
class TermiCfg:
    """Termination terms for the MDP."""
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, 
        params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object")} 
    )
    object_tipped = DoneTerm(
        func=mdp.bad_orientation,
        params={"limit_angle": 1.5, "asset_cfg": SceneEntityCfg("object")}, # 90 degrees ish
    )

    success = DoneTerm(func=mdp.object_reached_goal, params={"threshold": 0.05})

# --------------------------------------------------------------------------------
# region Reusable subclass cfg

@configclass
class BaseSubTaskCfg(SubTaskConfig):
    """ Inheriting and overwriting common configs """
    object_ref="object" # Only object in scene so is constant
    selection_strategy: str = "nearest_neighbor_object" # subtask strategy random/NNN_Object or NN_Robot 
    selection_strategy_kwargs: dict = {"nn_k": 1}
    action_noise: float = 0.0  # Less noise for this task


# --------------------------------------------------------------------------------
# region Task: Lift Cfg

@configclass
class FrankaIkLiftLabwareCfg(FrankaIkEnvCfg, MimicEnvCfg):
    """ Inherits Base & Ik Franka Scene Config from FrankaIkEnvCfg """
    
    observations: ObsCfg = ObsCfg()
    commands: CommandsCfg = CommandsCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TermiCfg = TermiCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    def __post_init__(self):
        super().__post_init__()
    
        self.commands.object_pose.body_name = "panda_hand"
        self.scene.object = Beaker.replace(prim_path="{ENV_REGEX_NS}/Object", pos=(0.4,0.25,0.02))
        self.scene.object.body_name = "Object"
        self.scene.hotplate = Hotplate.replace(
            prim_path="{ENV_REGEX_NS}/hotplate", pos=(goal_x, goal_y, 0.04), 
        )

        self.datagen_config.name = "demo_lift_labware_D0"
        self.datagen_config.generation_keep_failed = False
        self.datagen_config.generation_num_trials = 10
        self.datagen_config.generation_select_src_per_subtask = True
        self.datagen_config.generation_relative = True
        self.datagen_config.max_num_failures = 25

        self.subtask_configs["franka"] = [
            BaseSubTaskCfg(
                subtask_term_signal="appr",
                subtask_term_offset_range = (10, 20),
                description="Approach the labware",
                next_subtask_description="Grasp the labware",
            ),
            BaseSubTaskCfg(
                subtask_term_signal="grasp",
                subtask_term_offset_range = (5, 5),
                description="Grasp the labware",
                next_subtask_description="Lift the labware",
            ),
            BaseSubTaskCfg(
                subtask_term_signal="lift",
                subtask_term_offset_range = (5, 5),
                description="Lift the labware",
                next_subtask_description="Approach Goal Postiion", 
            ),
            BaseSubTaskCfg(
                subtask_term_signal="appr_goal",
                subtask_term_offset_range=(0, 0),   # 0,0 as last task
                description="Approach Goal Postiion",
                next_subtask_description=None,      # None as it's the final step
                
            ),
        ]


# --------------------------------------------------------------------------------
# region Task: Parallel Gripper Cfg

@configclass
class FrankaParallelIkLiftLabwareCfg(FrankaIkLiftLabwareCfg):
    """ Inherits Base & Ik Franka Scene Config from FrankaIkEnvCfg """

    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = PARALLEL_IK_FRANKA.replace(prim_path="{ENV_REGEX_NS}/Robot")
        