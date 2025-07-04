# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

from isaaclab_tasks.manager_based.manipulation.cube_lift import mdp
from isaaclab_tasks.manager_based.manipulation.cube_lift.lift_env_cfg import CubeEnvCfg

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip


@configclass
class FrankaLabEnvCfg(CubeEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

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
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "panda_hand"

        # Set Cube as object

        cube_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )

        # Set each stacking cube deterministically
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.4, 0.0, 0.0203], rot=[0, 1, 1, 0]),
            spawn=UsdFileCfg(
                # usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_glass.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=cube_properties,
                semantic_tags=[("class", "object")],
            ),
        )

        # ## Add sample vial
        # self.scene.samplevial = RigidObjectCfg(
        #     prim_path="{ENV_REGEX_NS}/Samplevial",
        #     init_state=RigidObjectCfg.InitialStateCfg(pos=[0.3, 0.0, 0.055], rot=[0, 1, 1, 0]),
        #     spawn=UsdFileCfg(
        #         usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_glass.usd",
        #         scale=(1, 1, 1),
        #         rigid_props=RigidBodyPropertiesCfg(),
        #     ),
        # )


        ## Add a beaker
        self.scene.beaker = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Beaker",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.6, 0, 0.055], rot=[0, 0, 1, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/glass_beaker_upright.usd",
                scale=(1, 1, 1),
                rigid_props=RigidBodyPropertiesCfg(),
            ),
        )

        ## Add a conical flask
        self.scene.conicalflask = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Conicalflask",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.3, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/conical_flask_100ml_glass_new.usd",
                scale=(1, 1, 1),
                rigid_props=RigidBodyPropertiesCfg(),
            ),
        )

        ## Add a second conical flask
        self.scene.secondflask = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Secondflask",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.7, 0.3, 0.055], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/second_conical_flask_100ml_glass_new.usd",
                scale=(1, 1, 1),
                rigid_props=RigidBodyPropertiesCfg(),
            ),
        )


        ## Add second sample vial
        self.scene.secondvial = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Secondvial",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.3, -0.2, 0.055], rot=[0, 1, 1, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/second_sample_vial_glass.usd",
                scale=(1, 1, 1),
                rigid_props=RigidBodyPropertiesCfg(),
            ),
        )

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
            ],
        )
