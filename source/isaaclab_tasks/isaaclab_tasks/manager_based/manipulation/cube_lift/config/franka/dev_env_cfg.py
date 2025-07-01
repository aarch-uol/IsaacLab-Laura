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
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
=======

>>>>>>> 23650e4deb (changes to scripts for logging)
=======
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
=======
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions.actions_cfg import DifferentialInverseKinematicsActionCfg
>>>>>>> b77a8f7870 (now with semi-working state machine!)
from isaaclab_tasks.manager_based.manipulation.cube_lift import mdp
from isaaclab_tasks.manager_based.manipulation.cube_lift.lift_env_cfg import CubeEnvCfg
from isaaclab_assets.glassware.glassware_objects import Chem_Assets
##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
from isaaclab_assets.robots.franka import FRANKA_PANDA_HIGH_PD_CFG  # isort: skip
=======
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip
>>>>>>> 23650e4deb (changes to scripts for logging)
=======
from isaaclab_assets.robots.franka import FRANKA_PANDA_HIGH_PD_CFG  # isort: skip
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
=======
from isaaclab_assets.robots.franka import FRANKA_PANDA_HIGH_PD_CFG  # isort: skip
>>>>>>> b77a8f7870 (now with semi-working state machine!)


@configclass
class FrankaDevEnvCfg(CubeEnvCfg):
    def __post_init__(self):
        # post init of parent
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> b77a8f7870 (now with semi-working state machine!)
       
        # post init of parent
        super().__post_init__()

        # Set Franka as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            body_name="panda_hand",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        )

<<<<<<< HEAD
=======
=======
       
        # post init of parent
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
        super().__post_init__()

        # Set Franka as robot
        # We switch here to a stiffer PD controller for IK tracking to be better.
        self.scene.robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = DifferentialInverseKinematicsActionCfg(
            asset_name="robot",
            joint_names=["panda_joint.*"],
            body_name="panda_hand",
            controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
            body_offset=DifferentialInverseKinematicsActionCfg.OffsetCfg(pos=[0.0, 0.0, 0.107]),
        )
<<<<<<< HEAD
>>>>>>> 23650e4deb (changes to scripts for logging)
=======

>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
=======
>>>>>>> b77a8f7870 (now with semi-working state machine!)
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(
            asset_name="robot",
            joint_names=["panda_finger.*"],
            open_command_expr={"panda_finger_.*": 0.04},
            close_command_expr={"panda_finger_.*": 0.0},
        )
        # Set the body name for the end effector
        self.commands.object_pose.body_name = "panda_hand"

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ec2c3a3a52a7b8ef6dfa907093f7abf8fe4c5831

        cube_properties = RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
=======
        # Set Cube as object

        #what if its now a vial rack  ?
=======
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
=======
>>>>>>> 5fa2eec84e (with state machine)

        cube_properties = RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
<<<<<<< HEAD
<<<<<<< HEAD
            max_depenetration_velocity=5.0,
            disable_gravity=False,
>>>>>>> 23650e4deb (changes to scripts for logging)
=======
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
=======
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
>>>>>>> 5fa2eec84e (with state machine)
        )

        # Set each stacking cube deterministically
        self.scene.object = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Object",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.4, 0.0, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/vial_rack.usd",
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
                scale=(0.75, 0.75, 1.5),
=======
                scale=(1.0, 1.0, 1.0),
>>>>>>> 23650e4deb (changes to scripts for logging)
=======
                scale=(0.75, 0.75, 1.5),
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
=======
                scale=(0.75, 0.75, 1.5),
>>>>>>> b77a8f7870 (now with semi-working state machine!)
                rigid_props=cube_properties,
                semantic_tags=[("class", "object")],
            ),
        )

        self.scene.flask = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/flask",
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.65, 0.4, 0.05],rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/conical_flask.usd",
                scale=(1, 1, 1),
                rigid_props=cube_properties,
                visible=True,
                copy_from_source = False,
=======
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.35, 0.05],rot=[1, 0, 0, 0]),
=======
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.65, 0.4, 0.05],rot=[1, 0, 0, 0]),
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/conical_flask.usd",
                scale=(1, 1, 1),
                rigid_props=cube_properties,
<<<<<<< HEAD
>>>>>>> 23650e4deb (changes to scripts for logging)
=======
                visible=True,
                copy_from_source = False,
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
=======
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.65, 0.4, 0.05],rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/conical_flask.usd",
                scale=(1, 1, 1),
<<<<<<< HEAD
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
>>>>>>> b77a8f7870 (now with semi-working state machine!)
=======
                rigid_props=cube_properties,
                visible=True,
                copy_from_source = False,
>>>>>>> 5fa2eec84e (with state machine)
                semantic_tags=[("class", "flask")],
            ),
        ) 

        self.scene.vial = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/vial",
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.65, 0.3, 0.05],rot=[0, 0, 1, 0]),
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_20ml.usd",
=======
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.65, 0.3, 0.05],rot=[0, 0, 1, 0]),
            spawn=UsdFileCfg(
<<<<<<< HEAD
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial.usd",
>>>>>>> b77a8f7870 (now with semi-working state machine!)
=======
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_20ml.usd",
>>>>>>> 5fa2eec84e (with state machine)
                scale=(1, 1, 1),
                rigid_props=cube_properties,
                visible=True,
                copy_from_source = False,
<<<<<<< HEAD
=======
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.6, 0.3, 0.05],rot=[0, 0, 1, 0]),
=======
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.65, 0.3, 0.05],rot=[0, 0, 1, 0]),
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_20ml.usd",
                scale=(1, 1, 1),
<<<<<<< HEAD
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
>>>>>>> 23650e4deb (changes to scripts for logging)
=======
                rigid_props=cube_properties,
                visible=True,
                copy_from_source = False,
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
=======
>>>>>>> 5fa2eec84e (with state machine)
                semantic_tags=[("class", "vial")],
            ),
        ) 
        self.scene.beaker = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/beaker",
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> b77a8f7870 (now with semi-working state machine!)
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.65, 0.2, 0.05],rot=[0, 0, 1, 0]),
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/beaker_500ml.usd",
                scale=(0.5, 0.5, 0.5),
<<<<<<< HEAD
<<<<<<< HEAD
                rigid_props=cube_properties,
=======
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
>>>>>>> b77a8f7870 (now with semi-working state machine!)
=======
                rigid_props=cube_properties,
>>>>>>> 5fa2eec84e (with state machine)
                semantic_tags=[("class", "beaker")],
            ),
        ) 
        self.scene.stirplate = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/stirplate",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.40, -0.3, 0.05],rot=[0.707, 0, 0, -0.707]),
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/lab_equipment/mag_hotplate.usd",
                scale=(0.8, 0.8, 0.8),
<<<<<<< HEAD
<<<<<<< HEAD
                rigid_props=cube_properties,
                semantic_tags=[("class", "stirplate")],
=======
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.55, 0.2, 0.05],rot=[0, 0, 1, 0]),
=======
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.65, 0.2, 0.05],rot=[0, 0, 1, 0]),
>>>>>>> 3add0cac05 (merged with IsaacLab-Laura)
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/beaker_500ml.usd",
                scale=(0.5, 0.5, 0.5),
                rigid_props=cube_properties,
                semantic_tags=[("class", "beaker")],
>>>>>>> 23650e4deb (changes to scripts for logging)
            ),
        ) 
        self.scene.stirplate = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/stirplate",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.40, -0.3, 0.05],rot=[0.707, 0, 0, -0.707]),
            spawn=UsdFileCfg(
                usd_path="/workspace/isaaclab/source/isaaclab_assets/data/Props/lab_equipment/mag_hotplate.usd",
                scale=(0.8, 0.8, 0.8),
                rigid_props=cube_properties,
=======
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
>>>>>>> b77a8f7870 (now with semi-working state machine!)
=======
                rigid_props=cube_properties,
>>>>>>> 5fa2eec84e (with state machine)
                semantic_tags=[("class", "stirplate")],
            ),
        ) 
=======
        glassware = Chem_Assets()
        # Set each stacking cube deterministically
        self.scene.object = glassware.beaker(pos=[0.4, 0.0, 0.0203],name="Object")
        #### everything else leave default
        # self.scene.flask = glassware.flask()
        # self.scene.vial = glassware.vial()
        # self.scene.beaker = glassware.beaker()
        self.scene.stirplate = glassware.stirplate()
        self.scene.random = glassware.random_object()
<<<<<<< HEAD
>>>>>>> 8222bb8ed3 (added scikit learn to build)
=======
>>>>>>> laura/laura-dev
>>>>>>> ec2c3a3a52a7b8ef6dfa907093f7abf8fe4c5831

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


@configclass
class FrankaDevEnvCfg_PLAY(FrankaDevEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        # make a smaller scene for play
        self.scene.num_envs = 50
        self.scene.env_spacing = 2.5
        # disable randomization for play
        self.observations.policy.enable_corruption = False
