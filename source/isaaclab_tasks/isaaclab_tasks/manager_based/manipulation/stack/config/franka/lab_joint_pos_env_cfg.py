# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.stack import mdp
from isaaclab_tasks.manager_based.manipulation.stack.mdp import franka_stack_events
from isaaclab_tasks.manager_based.manipulation.stack.lab_env_cfg import PourEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip
from isaaclab_assets.robots.franka import FRANKA_PANDA_HIGH_PD_CFG
from isaaclab.assets import ArticulationCfg
import math

from isaaclab.envs import ManagerBasedEnv

@configclass
class EventCfg:
    """Configuration for events."""

    # init_franka_arm_pose = EventTerm(
    #     func=franka_stack_events.set_default_joint_pose,
    #     mode="startup",
    #     params={
    #         "default_pose": [0.0444, -0.1894, -0.1107, -2.5148, 0.0044, 2.3775, 0.6952, 0.0400, 0.0400],
    #     },
    # )

    randomize_franka_joint_state = EventTerm(
        func=franka_stack_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    set_pose_event = EventTerm(
        func=franka_stack_events.set_default_joint_pose,
        mode="startup",  # or "reset"
        params={
            "default_pose": [0.405, 0.35, -0.22, -3.0, -2.85, math.pi/2, 0.9, 0.02, 0.02],
        },
    )

    reset_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.2, 0.10), "y": (-0.35, -0.175), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("hot_plate"),
        },
    )
    reset_object2_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.2, 0.15), "y": (0.3, 0.35), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object2"),
        },
    )
    # randomize_cube_positions = EventTerm(
    #     func=franka_stack_events.randomize_object_pose,
    #     mode="reset",
    #     params={
    #         "pose_range": {"x": (0.0, 0.2), "y": (0.0, 0.2), "z": (0.0203, 0.0203)},
    #         "min_separation": 0.1,
    #         "asset_cfgs": [SceneEntityCfg("object1")] 
    #     },
    # )
    randomize_object1_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.2, 0.25), "y": (-0.05, 0.125), "z": (0.0203, 0.0203)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("object1"),
        },
    )

    # If want to manually overide the scale, this needs to be a part of the function, have a param=None and an if statement
    # This has to be done with replicate_physics=False and prestartup, this only gives random before scene created
    randomize_scale = EventTerm(
        func=mdp.randomize_rigid_body_scale,
        mode="prestartup",
        params={
            "scale_range": (0.8, 1.2),
            "asset_cfg": SceneEntityCfg("object1"),
        },
    )


@configclass
class FrankaLabStackEnvCfg(PourEnvCfg): # StackEnvCfg
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set events
        self.events = EventCfg()

        # Set Franka as robot
        # self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(
            prim_path="{ENV_REGEX_NS}/Robot",
            init_state=ArticulationCfg.InitialStateCfg(
                joint_pos={
                    "panda_joint1":  0.405,
                    "panda_joint2": 0.35,   
                    "panda_joint3":  -0.22, 
                    "panda_joint4": -3.0,
                    "panda_joint5":  -2.85, 
                    "panda_joint6":  math.pi / 2,  #  +90° → keeps hand level
                    "panda_joint7":  0.9, 
                    "panda_finger_joint1": 0.04,   # open gripper
                    "panda_finger_joint2": 0.04,
                }
            ),
        )

        self.scene.robot.spawn.semantic_tags = [("class", "robot")]

        # Add semantics to table
        self.scene.table.spawn.semantic_tags = [("class", "table")]

        # Add semantics to ground
        self.scene.plane.semantic_tags = [("class", "ground")]

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = mdp.JointPositionActionCfg(
            asset_name="robot", joint_names=["panda_joint.*"], scale=0.5, use_default_offset=True
        )
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["panda_finger.*"],
            open_command_expr={"panda_finger_.*": 0.04},
            close_command_expr={"panda_finger_.*": 0.0},
        )


        self.commands.object_pose.body_name = "panda_hand"

        # Rigid body properties of each cube
        cube_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )

        self.scene.hot_plate = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Hot_plate",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.0, 0.0203], rot=[0.707, 0.707, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/lab_equipment/hot_plate.usd",
                scale=(0.03, 0.03, 0.03),
                rigid_props=cube_properties,
                semantic_tags=[("class", "hot_plate")],
            ),
        )
        self.scene.object1 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object1",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.0, 0.0203], rot=[1, 0, 0, 0]),
            # init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.0, 0.0203], rot=[0.707, 0.707, 0, 0]),
            spawn=UsdFileCfg(
                # usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_glass.usd",
                # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                # usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/centre_beaker.usd",
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/beaker_new.usd",
                # usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/centered_conical_flask_glass.usd",
                # usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_glass.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=RigidBodyPropertiesCfg(),
                semantic_tags=[("class", "object1")],
            ),
        )
        ### As soon as I create this usd, hot plate doesn't work and vice versa
        self.scene.object2 = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Electronic_balance",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.0, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/scale/balance_electronic.usd",
                scale=(0.01, 0.01, 0.01),
                rigid_props=cube_properties,
                semantic_tags=[("class", "electronic_balance")],
            ),
        )
        # self.scene.funnel = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Funnel",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.55, -0.2, 0.0203], rot=[0.707, 0.707, 0, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/funnel.usd",
        #         scale=(0.02, 0.02, 0.02),
        #         rigid_props=cube_properties,
        #         semantic_tags=[("class", "funnel")],
        #     ),
        # )
        # self.scene.tube_rack = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Tube_rack",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.85, -0.1, 0.0203], rot=[0.707, 0.707, 0, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/test_tube_rack/test_tube_rack.usd",
        #         scale=(0.03, 0.03, 0.03),
        #         rigid_props=cube_properties,
        #         semantic_tags=[("class", "tube_rack")],
        #     ),
        # )

        # Listens to the required transforms
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                    name="end_effector",
                    offset=OffsetCfg(
                        pos=[0.0, 0.0, 0.1034],
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
                    name="tool_rightfinger",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.046),
                    ),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
                    name="tool_leftfinger",
                    offset=OffsetCfg(
                        pos=(0.0, 0.0, 0.046),
                    ),
                ),
            ],
        )

        # marker_cfg = FRAME_MARKER_CFG.copy()
        # marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        # marker_cfg.prim_path = "/Visuals/FrameTransformer"
        # self.scene.ee_frame = FrameTransformerCfg(
        #     prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
        #     debug_vis=False,
        #     visualizer_cfg=marker_cfg,
        #     target_frames=[
        #         FrameTransformerCfg.FrameCfg(
        #             prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
        #             name="end_effector",
        #             offset=OffsetCfg(
        #                 pos=[0.0, 0.0, 0.1034],
                        
        #             ),
        #         ),
        #     ],
        # )