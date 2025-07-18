# File: source/chills/chills/envs/ur10_cfg.py
# --------------------------------------------------------------------------------


import math
from isaaclab.utils import configclass
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.envs.mdp.actions.actions_cfg import BinaryJointPositionActionCfg, DifferentialInverseKinematicsActionCfg
from isaaclab.controllers.differential_ik_cfg import DifferentialIKControllerCfg


# Work in progress: This is a temporary UR10 configuration for testing purposes.


UR10e_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path="assets/usds/ur10e_robotiq2f-140.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 0.0,              #Base left right
            "shoulder_lift_joint": math.radians(75),# Arm up down
            "elbow_joint": math.radians(155),       # Kinda forwards backwards with lift joint
            "wrist_1_joint": math.radians(-74),     # EE up down
            "wrist_2_joint": math.radians(90),      # EE left right
            "wrist_3_joint": 0.0,

        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["shoulder_.*", "elbow_.*", "wrist_.*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
        "gripper": ImplicitActuatorCfg(
            joint_names_expr=["robotiq_.*"],
            effort_limit=20.0,
            stiffness=1000.0,
            damping=100.0,
        ),
    },
)

@configclass
class UR10eActionsCfg:
    arm_action = DifferentialInverseKinematicsActionCfg(
        asset_name="robot",
        joint_names=["shoulder_.*", "elbow_.*", "wrist_.*"],
        body_name="robotiq_base_link",  
        controller=DifferentialIKControllerCfg(
            command_type="pose",
            use_relative_mode=True,
            ik_method="dls",
        ),
        scale=0.5,
        use_default_offset=True,
    )

    gripper_action = BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["robotiq_.*"],
        open_command_expr={"robotiq_85_left_knuckle_joint": 0.8},  # fully open
        close_command_expr={"robotiq_85_left_knuckle_joint": 0.0},  # fully closed
    )