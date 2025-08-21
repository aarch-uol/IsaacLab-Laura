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
from isaaclab_tasks.manager_based.manipulation.stack.stack_env_cfg import StackEnvCfg

from isaaclab.markers.config import FRAME_MARKER_CFG  
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  

@configclass
class EventCfg:
    """Configuration for events."""

    init_franka_arm_pose = EventTerm(
        func=franka_stack_events.set_default_joint_pose,
        mode="startup",
        params={
            "default_pose": [0.0444, -0.1894, -0.1107, -2.5148, 0.0044, 2.3775, 0.6952, 0.0400, 0.0400],
        },
    )

    randomize_franka_joint_state = EventTerm(
        func=franka_stack_events.randomize_joint_state,
        mode="reset",
        params={
            "joints": ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
            "min": [-2.0, -1.5, -2.0, -3.0, -1.5, -3.0],
            "max": [2.0, 1.5, 2.0, 3.0, 1.5, 3.0],
        },
    )

@configclass
class FrankaCubeStackEnvCfg(StackEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        
        # Robot configuration
        self.scene.robot = FRANKA_PANDA_CFG.replace(
            {
                "prim_path": "/Robot",
                "joint_stiffness": [1000.0]*7,
                "joint_damping": [100.0]*7,
            }
        )

        # Cube configuration
        self.scene.cube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube",
            init_state=dict(
                pos=[0.5, 0.0, 0.6],
                rot=[1.0, 0.0, 0.0, 0.0],
                vel=[0.0, 0.0, 0.0],
                ang_vel=[0.0, 0.0, 0.0],
            ),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(4.0, 4.0, 1.0),
                rigid_body_props=RigidBodyPropertiesCfg(
                    inertia=[0.001, 0.001, 0.001],
                ),
                visual_material=None,
                collision_material=None,
                override_material=None,
            ),
            physics_material=None,
            rigid_body_props=None,
            visualizer=None,
        )

        # Beaker configuration
        self.scene.beaker = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Beaker",
            init_state=dict(
                pos=[-0.5, 0.0, 0.6],
                rot=[1.0, 0.0, 0.0, 0.0],
                vel=[0.0, 0.0, 0.0],
                ang_vel=[0.0, 0.0, 0.0],
            ),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/glass_beaker_upright.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_body_props=RigidBodyPropertiesCfg(
                    inertia=[0.001, 0.001, 0.001],
                ),
                visual_material=None,
                collision_material=None,
                override_material=None,
            ),
            physics_material=None,
            rigid_body_props=None,
            visualizer=None,
        )

        # End-effector frame transformer
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=FRAME_MARKER_CFG.replace(
                {
                    "prim_path": "/Visuals/FrameTransformer",
                    "visualizer_color": [0.0, 1.0, 0.0, 1.0],
                }
            ),
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
