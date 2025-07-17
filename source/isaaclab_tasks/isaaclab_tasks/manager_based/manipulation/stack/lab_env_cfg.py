# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.devices.openxr import XrCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from . import mdp


##
# Scene definition
##
@configclass
class ObjectTableSceneCfg(InteractiveSceneCfg):
    """Configuration for the lift scene with a robot and a object.
    This is the abstract base implementation, the exact scene is defined in the derived classes
    which need to set the target object, robot and end-effector frames
    """

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by agent env cfg
    ee_frame: FrameTransformerCfg = MISSING

    # Table
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd"),
    )

    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
    )

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )


##
# MDP settings
##
@configclass
class CommandsCfg:
    """Command terms for the MDP."""
#### Goal Positions
    object_pose = mdp.UniformPoseCommandCfg(
        asset_name="robot",
        body_name=MISSING,  # will be set by agent env cfg
        resampling_time_range=(10.0, 10.0),
        debug_vis=True,
        ranges=mdp.UniformPoseCommandCfg.Ranges(
            pos_x=(0.4,0.4), pos_y=(-0.25, -0.25), pos_z=(0.5, 0.5), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
        ),
    )

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    # will be set by agent env cfg
    arm_action: mdp.JointPositionActionCfg = MISSING
    gripper_action: mdp.BinaryJointPositionActionCfg = MISSING


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
        cube_positions = ObsTerm(func=mdp.cube_positions_in_world_frame)
        cube_orientations = ObsTerm(func=mdp.cube_orientations_in_world_frame)
        eef_pos = ObsTerm(func=mdp.ee_frame_pos)
        eef_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class SubtaskCfg(ObsGroup):
        """Observations for subtask group."""
        ### Is there a way to make multiple subtaskcfgs
        appr_obj1 = ObsTerm(
            func=mdp.reach_object,
            params={
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("object1"),
                "std" : 0.05
            }
        )
        grasp_obj1 = ObsTerm(
            func=mdp.object_grasped,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("object1"),
            },
        )
        lift = ObsTerm(
            func=mdp.is_object_lifted,
            params={
                "obj_cfg": SceneEntityCfg("object1"),
                "threshold" : 0.1
            }
        )
        ### Add a waypoint
        appr_obj2 = ObsTerm(
            func=mdp.reach_object2,
            params={
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object2_cfg": SceneEntityCfg("object2"),
                "std" : 0.05
            }
        )
        stack = ObsTerm(
            func=mdp.object_stacked,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "upper_object_cfg": SceneEntityCfg("object1"),
                "lower_object_cfg": SceneEntityCfg("object2"),
            },
        )


        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    subtask_terms: SubtaskCfg = SubtaskCfg()

@configclass
class Observations2Cfg:
    """Observation specifications for the MDP."""

    @configclass
    class Policy2Cfg(ObsGroup):
        """Observations for policy group with state values."""

        actions = ObsTerm(func=mdp.last_action)
        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object = ObsTerm(func=mdp.object_obs)
        cube_positions = ObsTerm(func=mdp.cube_positions_in_world_frame)
        cube_orientations = ObsTerm(func=mdp.cube_orientations_in_world_frame)
        eef_pos = ObsTerm(func=mdp.ee_frame_pos)
        eef_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class Subtask2Cfg(ObsGroup):
        """Observations for subtask group."""

        # def set_loghelper(self, loghelper: LoggingHelper):
            # Inject the logger into any terms that need it
            # self.appr.params["loghelper"] = loghelper
            # self.grasp.params["loghelper"] = loghelper
            # self.lift.params["loghelper"] = loghelper
            # self.appr_goal.params["loghelper"] = loghelper
            # self.loghelper = loghelper


        appr = ObsTerm(
            func=mdp.reach_object,
            params={
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("object1"),
                "std" : 0.05
            }
        )
        
        grasp = ObsTerm(
            func=mdp.object_grasped,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("object1"),
            },
        )
        lift = ObsTerm(
            func=mdp.is_object_lifted,
            params={
                "obj_cfg": SceneEntityCfg("object1"),
                "threshold" : 0.1
            }
        )
        appr_midgoal = ObsTerm(
            func=mdp.object_reached_midgoal,
            params={ 
                "threshold": 0.04, 
                "command_name": "object_pose",
                
            },
        )
        appr_obj2 = ObsTerm(
            func=mdp.reach_object2,
            params={
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object2_cfg": SceneEntityCfg("object2"),
                "std" : 0.05
            }
        )
        pour = ObsTerm(
            func=mdp.pour_object, 
            params={
                "hand_frame_cfg": SceneEntityCfg("ee_frame"),
                "angle_threshold": 45,
            }
        )
        appr_goal = ObsTerm() # on table?
        #     func=mdp.object_near_goal,
        #     params={ 
        #         "threshold": 0.04, 
        #         "command_name": "object_pose",
                
        #     },
        # )
        
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False  

    policy: Policy2Cfg = Policy2Cfg()
    subtask2_terms: Subtask2Cfg = Subtask2Cfg()

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_1_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object1")} 
    )
    object_2_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object2")}
    )
    ### OBJECT1 DROPPED - CONICAL
    robot_drop_object = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": 0.005, "asset_cfg": SceneEntityCfg("object1")}
    )
    success_stack = DoneTerm(func=mdp.objects_stacked)

@configclass
class Terminations2Cfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    object_1_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object1")} 
    )
    object_2_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object2")}
    )
    ### OBJECT1 DROPPED - CONICAL
    # success_lift = DoneTerm(func=mdp.object_reached_goal)


@configclass
class StackEnvCfg(ManagerBasedRLEnvCfg): # Could make multiple of these and get the LLM to choose from it
    """Configuration for the stacking environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=4096, env_spacing=2.5, replicate_physics=False)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg() # Subtasks is a part of this
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()

    # Unused managers
    rewards = None
    events = None # Elsewhere
    curriculum = None

    xr: XrCfg = XrCfg(
        anchor_pos=(-0.1, -0.5, -1.05),
        anchor_rot=(0.866, 0, 0, -0.5),
    )

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 5
        self.episode_length_s = 80.0 # used to be 30.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = 2

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

@configclass
class PourEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the pouring environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=4096, env_spacing=2.5, replicate_physics=False)
    # Basic settings
    observations: Observations2Cfg = ObservationsCfg() # Subtasks is a part of this
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    terminations: Terminations2Cfg = Terminations2Cfg()
    commands: CommandsCfg = CommandsCfg()

    # Unused managers
    rewards = None
    events = None # Elsewhere
    curriculum = None

    xr: XrCfg = XrCfg(
        anchor_pos=(-0.1, -0.5, -1.05),
        anchor_rot=(0.866, 0, 0, -0.5),
    )

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 5
        self.episode_length_s = 80.0 # used to be 30.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = 2

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
