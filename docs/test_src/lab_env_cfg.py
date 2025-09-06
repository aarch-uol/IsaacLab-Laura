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
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab_tasks.manager_based.manipulation.stack.mdp import franka_stack_events
import math
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
# from isaaclab.utils.logging_helper import LoggingHelper, ErrorType, LogType

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
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, 0], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/ThorlabsTable/table_instanceable.usd"),
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
            pos_x=(0.4, 0.4), pos_y=(-0.1, -0.1), pos_z=(0.15, 0.15), roll=(0.0, 0.0), pitch=(0.0, 0.0), yaw=(0.0, 0.0)
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
        object_positions = ObsTerm(func=mdp.object_positions_in_world_frame)
        object_orientations = ObsTerm(func=mdp.object_orientations_in_world_frame)
        eef_pos = ObsTerm(func=mdp.ee_frame_pos)
        eef_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False

    @configclass
    class SubtaskCfg(ObsGroup):
        """Observations for subtask group."""
        # def set_loghelper(self, loghelper: LoggingHelper):
        #     # Inject the logger into any terms that need it
        #     self.appr_obj1.params["loghelper"] = loghelper
        #     self.grasp_obj1.params["loghelper"] = loghelper
        #     self.lift.params["loghelper"] = loghelper
        #     self.appr_midgoal.params["loghelper"] = loghelper
        #     self.appr_obj2.params["loghelper"] = loghelper
        #     self.stack.params["loghelper"] = loghelper
            # self.loghelper = loghelper

        # appr_obj1 = ObsTerm(
        #     func=mdp.reach_object,
        #     params={
        #         "ee_frame_cfg": SceneEntityCfg("ee_frame"),
        #         "object_cfg": SceneEntityCfg("object1"),
        #         "threshold" : 0.05
        #     }
        # )
        # grasp_obj1 = ObsTerm(
        #     func=mdp.object_grasped,
        #     params={
        #         "robot_cfg": SceneEntityCfg("robot"),
        #         "ee_frame_cfg": SceneEntityCfg("ee_frame"),
        #         "object_cfg": SceneEntityCfg("object1"),
        #     },
        # )
        # lift = ObsTerm(
        #     func=mdp.is_object_lifted,
        #     params={
        #         "object_cfg": SceneEntityCfg("object1"),
        #         "threshold" : 0.1
        #     }
        # )
        # ### Check
        # appr_midgoal = ObsTerm(
        #     func=mdp.object_reached_midgoal,
        #     params={ 
        #         "threshold": 0.04, 
        #         "command_name": "object_pose",
                
        #     },
        # )
        # appr_obj2 = ObsTerm(
        #     func=mdp.reach_object2,
        #     params={
        #         "ee_frame_cfg": SceneEntityCfg("ee_frame"),
        #         "object2_cfg": SceneEntityCfg("object2"),
        #         "threshold" : 0.05
        #     }
        # )
        # stack = ObsTerm(
        #     func=mdp.object_stacked,
        #     params={
        #         "robot_cfg": SceneEntityCfg("robot"),
        #         "upper_object_cfg": SceneEntityCfg("object1"),
        #         "lower_object_cfg": SceneEntityCfg("object2"),
        #     },
        # )


        # def __post_init__(self):
        #     self.enable_corruption = False
        #     self.concatenate_terms = False

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    #subtask_terms: SubtaskCfg = SubtaskCfg()

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
        cube_positions = ObsTerm(func=mdp.object_positions_in_world_frame)
        cube_orientations = ObsTerm(func=mdp.object_orientations_in_world_frame)
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
                "threshold" : 0.05
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
                "object_cfg": SceneEntityCfg("object1"),
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
                "threshold" : 0.05
            }
        )
        pour = ObsTerm(
            func=mdp.pouring_solution, 
            params={
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "angle_threshold": 45,
            }
        )
        reorient = ObsTerm(
            func=mdp.reorient_object,
            params={
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "angle_threshold": 175,
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

    policy: Policy2Cfg = Policy2Cfg()
    subtask2_terms: Subtask2Cfg = Subtask2Cfg()

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    # def set_loghelper(self, loghelper: LoggingHelper):    
    #     self.success_stack.params["loghelper"] = loghelper
    #     self.time_out.params["loghelper"] = loghelper
    #     self.object_1_dropping.params["loghelper"] = loghelper
    #     self.object_2_dropping.params["loghelper"] = loghelper
    #     self.robot_drop_dropping.params["loghelper"] = loghelper

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
    # success_stack = DoneTerm(func=mdp.ObjectsStacked(
    #     lower_object_cfg=SceneEntityCfg("object2"),
    #     object_1_cfg=SceneEntityCfg("object1"),
    #     success_hold_steps = 150, 
    # ))

    # success = DoneTerm(func=mdp.ObjectsStacked(
    #     lower_object_cfg=SceneEntityCfg("object2"),
    #     object_1_cfg=SceneEntityCfg("object1"),
    #     success_hold_steps = 150, 
    # ))



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
    success_goal = DoneTerm(func=mdp.object_reached_goal)

@configclass
class EventCfg:
    """Configuration for events."""

    init_franka_arm_pose = EventTerm(
        func=franka_stack_events.set_default_joint_pose,
        mode="startup",
        params={
            "default_pose": [0.405, 0.35, -0.22, -3.0, -2.85, math.pi/2, 0.9, 0.02, 0.02],
        },
    )
    randomize_franka_joint_state = EventTerm(
        func=franka_stack_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )
    # reset_object1_position = EventTerm(
    #     func=mdp.reset_root_state_uniform,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.4, 0.6), "y": (-0.3, -0.05), "z": (0.0, 0.0)},
    #         "velocity_range": {},
    #         "asset_cfg": SceneEntityCfg("object1"), 
    #     },
    # )
    reset_object_2_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (0.4, 0.5), "y": (0.1, 0.4), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object2"),
        },
    )
    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    # reset_object_position = EventTerm(
    #     func=mdp.reset_root_state_uniform,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0, 0.2), "y": (0, 0.25), "z": (0.0, 0.0)},
    #         "velocity_range": {},
    #         "asset_cfg": SceneEntityCfg("object1", body_names="Beaker"),
    #     },
    # )
    reset_objects_position = EventTerm(
        func=mdp.reset_root_state_uniform_many,
        mode="reset",
        params={
            "pose_range": {"x": (0, 0.2), "y": (0, 0.25), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object1"),
            #"asset2_cfg": SceneEntityCfg("ball_0", body_names="ball_0"),
        },
    )

    # reset_object3_position = EventTerm(
    #     func=mdp.reset_root_state_uniform,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.0, 0.1), "y": (-0.25, -0.20), "z": (0.0, 0.0)},
    #         "velocity_range": {},
    #         "asset_cfg": SceneEntityCfg("object3"),
    #     }
    # )


@configclass
class StackEnvCfg(ManagerBasedRLEnvCfg): # Could make multiple of these and get the LLM to choose from it
    """Configuration for the stacking environment."""

    # Scene settings
    scene: ObjectTableSceneCfg = ObjectTableSceneCfg(num_envs=4096, env_spacing=2.5, replicate_physics=False)
    # Basic settings
    # observations: ObservationsCfg = ObservationsCfg() # Subtasks is a part of this
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    # terminations: TerminationsCfg = TerminationsCfg()
    commands: CommandsCfg = CommandsCfg()
    events: EventCfg = EventCfg()
    # loghelper = LoggingHelper()

    # Unused managers
    rewards = None
    curriculum = None
    terminations = None # Elsewhere
    observations = None # Elsewhere

    xr: XrCfg = XrCfg(
        anchor_pos=(-0.1, -0.5, -1.05),
        anchor_rot=(0.866, 0, 0, -0.5),
    )

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 5
        self.episode_length_s = 250 #300.0
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = 5

        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
        self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
        self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625

