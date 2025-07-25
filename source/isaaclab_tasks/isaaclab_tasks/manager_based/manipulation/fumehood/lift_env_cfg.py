# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import MISSING
from enum import Enum
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, DeformableObjectCfg, RigidObjectCfg, RigidObjectCollection, RigidObjectCollectionCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import CurriculumTermCfg as CurrTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import FrameTransformerCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import GroundPlaneCfg, UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
import warp as wp
from . import mdp
from isaaclab.utils.logging_helper import LoggingHelper, ErrorType, LogType

# wp.init()
# ## state machine config 
# class PickSmState:
#     """States for the pick state machine."""
#     REST = wp.constant(0)
#     APPR = wp.constant(1)
#     GRASP = wp.constant(2)
#     LIFT = wp.constant(3)
#     APP_GOAL = wp.constant(4)

# class PickSmWaitTime:
#     """Additional wait times (in s) for states for before switching."""

#     REST = wp.constant(0.5)
#     APPR = wp.constant(0.5)
#     GRASP = wp.constant(0.5)
#     LIFT = wp.constant(0.5)
#     APPR_GOAL = wp.constant(0.5)

# class GripperState:
#     """States for the gripper."""

#     OPEN = wp.constant(1.0)
#     CLOSE = wp.constant(-1.0)

# @wp.kernal
# def infer_state(
#     dt: wp.array(dtype=float),
#     sm_state: wp.array(dtype=int),
#     sm_wait_time: wp.array(dtype=float),
#     ee_pose: wp.array(dtype=wp.transform),
#     object_pose: wp.array(dtype=wp.transform),
#     des_object_pose: wp.array(dtype=wp.transform),
#     des_ee_pose: wp.array(dtype=wp.transform),
#     gripper_state: wp.array(dtype=float),
#     offset: wp.array(dtype=wp.transform),
# ):
#     tid = wp.tid()
#     state = sm_state[tid]
#     # decide next state
#     if state == PickSmState.REST:
#         des_ee_pose[tid] = ee_pose[tid]
#         gripper_state[tid] = GripperState.OPEN
#         # wait for a while
#         if sm_wait_time[tid] >= PickSmWaitTime.REST:
#             # move to next state and reset wait time
#             sm_state[tid] = PickSmState.APPR
#             sm_wait_time[tid] = 0.0
#     elif state == PickSmState.APPR:
#         des_ee_pose[tid] = wp.transform_multiply(offset[tid], object_pose[tid])
#         gripper_state[tid] = GripperState.OPEN
#         # TODO: error between current and desired ee pose below threshold

#         # wait for a while
#         if sm_wait_time[tid] >= PickSmWaitTime.APPROACH_OBJECT:
#             # move to next state and reset wait time
#             sm_state[tid] = PickSmState.APPROACH_OBJECT
#             sm_wait_time[tid] = 0.0
#     elif state == PickSmState.APPROACH_OBJECT:
#         des_ee_pose[tid] = object_pose[tid]
#         gripper_state[tid] = GripperState.OPEN
#         # TODO: error between current and desired ee pose below threshold
#         # wait for a while
#         if sm_wait_time[tid] >= PickSmWaitTime.APPROACH_OBJECT:
#             # move to next state and reset wait time
#             sm_state[tid] = PickSmState.GRASP_OBJECT
#             sm_wait_time[tid] = 0.0
#     elif state == PickSmState.GRASP_OBJECT:
#         des_ee_pose[tid] = object_pose[tid]
#         gripper_state[tid] = GripperState.CLOSE
#         # wait for a while
#         if sm_wait_time[tid] >= PickSmWaitTime.GRASP_OBJECT:
#             # move to next state and reset wait time
#             sm_state[tid] = PickSmState.LIFT_OBJECT
#             sm_wait_time[tid] = 0.0
#     elif state == PickSmState.LIFT_OBJECT:
#         des_ee_pose[tid] = des_object_pose[tid]
#         gripper_state[tid] = GripperState.CLOSE
#         # TODO: error between current and desired ee pose below threshold
#         # wait for a while
#         if sm_wait_time[tid] >= PickSmWaitTime.LIFT_OBJECT:
#             # move to next state and reset wait time
#             sm_state[tid] = PickSmState.LIFT_OBJECT
#             sm_wait_time[tid] = 0.0
#     # increment wait time
#     sm_wait_time[tid] = sm_wait_time[tid] + dt[tid]




##
# Scene definition
##


@configclass
class FumehoodSceneCfg(InteractiveSceneCfg):
   
   

    # robots: will be populated by agent env cfg
    robot: ArticulationCfg = MISSING
    # end-effector sensor: will be populated by agent env cfg
    ee_frame: FrameTransformerCfg = MISSING
    # target object: will be populated by agent env cfg
    object: RigidObjectCfg | DeformableObjectCfg = MISSING
    
    
   

   # glassware = RigidObjectCollectionCfg(rigid_objects={"vial" : vial, "flask" : flask})


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
    arm_action: mdp.JointPositionActionCfg | mdp.DifferentialInverseKinematicsActionCfg = MISSING
    gripper_action: mdp.BinaryJointPositionActionCfg = MISSING


@configclass
class ObservationsCfg():
    """Observation specifications for the MDP."""
    
    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        joint_pos = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel = ObsTerm(func=mdp.joint_vel_rel)
        object_position = ObsTerm(func=mdp.object_position_in_robot_root_frame)
        target_object_position = ObsTerm(func=mdp.generated_commands, params={"command_name": "object_pose"})
        actions = ObsTerm(func=mdp.last_action)
        object_to_target = ObsTerm(func=mdp.object_reached_goal)
        eef_pos = ObsTerm(func=mdp.ee_frame_pos)
        eef_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)
        
        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False
    
    @configclass
    class SubtaskCfg(ObsGroup):
        """Observations for subtask group."""
        def set_loghelper(self, loghelper: LoggingHelper):
            # Inject the logger into any terms that need it
            self.appr.params["loghelper"] = loghelper
            self.grasp.params["loghelper"] = loghelper
            self.lift.params["loghelper"] = loghelper
            self.appr_goal.params["loghelper"] = loghelper
           # self.loghelper = loghelper


        appr = ObsTerm(
            func=mdp.reach_object,
            params={
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("object"),
                "std" : 0.05
            }
        )
        
        grasp = ObsTerm(
            func=mdp.object_grasped,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "ee_frame_cfg": SceneEntityCfg("ee_frame"),
                "object_cfg": SceneEntityCfg("object"),
            },
        )
        lift = ObsTerm(
            func=mdp.is_object_lifted,
            params={
                "obj_cfg": SceneEntityCfg("object"),
                "threshold" : 0.1
            }
        )
        appr_goal = ObsTerm(
            func=mdp.object_near_goal,
            params={ 
                "threshold": 0.04, 
                "command_name": "object_pose",
                
            },
        )
     

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = False
            
            
            

    # observation groups
    policy: PolicyCfg = PolicyCfg()
    subtask_terms : SubtaskCfg = SubtaskCfg()

@configclass
class EventCfg:
    """Configuration for events."""

    reset_all = EventTerm(func=mdp.reset_scene_to_default, mode="reset")

    reset_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.1, 0.1), "y": (-0.25, 0.25), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object", body_names="Object"),
        },
    )

    reset_task = EventTerm(func = mdp.randomize_object, mode="reset")

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    reaching_object = RewTerm(func=mdp.object_ee_distance, params={"std": 0.1}, weight=1.0)

    lifting_object = RewTerm(func=mdp.object_is_lifted, params={"minimal_height": 0.14}, weight=30.0)

   # reaching_target = RewTerm(func=mdp.object_goal_distance, weight = 20.0)

    object_goal_tracking = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.3, "minimal_height": 0.04, "command_name": "object_pose"},
        weight=16.0,
    )

    object_goal_tracking_fine_grained = RewTerm(
        func=mdp.object_goal_distance,
        params={"std": 0.05, "minimal_height": 0.04, "command_name": "object_pose"},
        weight=5.0,
    )

    # action penalty
    action_rate = RewTerm(func=mdp.action_rate_l2, weight=-1e-4)

    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-1e-4,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    def set_loghelper(self, loghelper: LoggingHelper):    
        self.success.params["loghelper"] = loghelper
        self.time_out.params["loghelper"] = loghelper
        self.object_dropping.params["loghelper"] = loghelper
        #self.loghelper = loghelper

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    joint_violation = DoneTerm(func=mdp.joint_pos_out_of_limit)
    
   # object_orientation = DoneTerm(func=mdp.bad_orientation)

   # joint_effort = DoneTerm(func=mdp.joint_effort_out_of_limit)

    object_dropping = DoneTerm(
        func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object")}
    )
    #fix object dropping
    success = DoneTerm(func=mdp.object_reached_goal)


@configclass
class CurriculumCfg:
    """Curriculum terms for the MDP."""

    action_rate = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "action_rate", "weight": -1e-1, "num_steps": 10000}
    )

    joint_vel = CurrTerm(
        func=mdp.modify_reward_weight, params={"term_name": "joint_vel", "weight": -1e-1, "num_steps": 10000}
    )


##
# Environment configuration
##



@configclass
class FumehoodEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the lifting environment."""
    loghelper = LoggingHelper()
    # Scene settings
    scene: FumehoodSceneCfg = FumehoodSceneCfg(num_envs=4096, env_spacing=2.5)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    commands: CommandsCfg = CommandsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    events: EventCfg = EventCfg()
    curriculum: CurriculumCfg = CurriculumCfg()

    #subtask_statemachine: SubtaskStateMachine = SubtaskStateMachine()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 60
        # simulation settings
        self.sim.dt = 0.01  # 100Hz
        self.sim.render_interval = self.decimation
        self.viewer.eye = (0.5, 0.5, 0.5)
        self.viewer.lookat = (0.0, 0.0, -0.1)
        self.terminations.set_loghelper(self.loghelper)
        self.observations.subtask_terms.set_loghelper(self.loghelper)
        self.sim.physx.bounce_threshold_velocity = 0.2
        self.sim.physx.bounce_threshold_velocity = 0.01
     #   self.sim.physx.gpu_found_lost_aggregate_pairs_capacity = 1024 * 1024 * 4
       # self.sim.physx.gpu_total_aggregate_pairs_capacity = 16 * 1024
        self.sim.physx.friction_correlation_distance = 0.00625
