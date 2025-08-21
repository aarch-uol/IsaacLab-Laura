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
        func=franka_stack_events.randomize_joint_by_gaussian_offset,
        mode="reset",
        params={
            "mean": 0.0,
            "std": 0.02,
            "asset_cfg": SceneEntityCfg("robot"),
        },
    )

    randomize_cube_positions = EventTerm(
        func=franka_stack_events.randomize_object_pose,
        mode="reset",
        params={
            "pose_range": {"x": (0.4, 0.6), "y": (-0.10, 0.10), "z": (0.0203, 0.0203), "yaw": (-1.0, 1, 0)},
            "min_separation": 0.1,
            "asset_cfgs": [SceneEntityCfg("cube_3"), SceneEntityCfg("cube_4")],
        },
    )

    reset_object_position = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {"x": (-0.1, 0.1), "y": (-0.25, 0.25), "z": (0.0, 0.0)},
            "velocity_range": {},
            "asset_cfg": SceneEntityCfg("cube_2", body_names="Sample_vial"),
        },
    )

@configclass
class FrankaCubeStackEnvCfg(StackEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.events = EventCfg()

        self.scene.robot = FRANKA_PANDA_CFG

        self.scene.cube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube",
            name="cube",
            rigid_props=RigidBodyPropertiesCfg(
                max_angular_velocity=3.14,
                max_linear_velocity=10.0,
            ),
            spawn=UsdFileCfg(
                usd_path="omni.isaac.robots/Assets/Blocks/red_block.usd",
                rigid_body_props=RigidBodyPropertiesCfg(
                    max_angular_velocity=3.14,
                    max_linear_velocity=10.0,
                ),
            ),
            physics_material=PhysicsMaterialCfg(
                static_friction=0.5,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
            collision_props=CollisionPropertiesCfg(
                enabled=True,
                allowed_collision_filters=["franka", "cube"],
            ),
        )

        self.scene.beaker = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Beaker",
            name="beaker",
            rigid_props=RigidBodyPropertiesCfg(
                max_angular_velocity=3.14,
                max_linear_velocity=10.0,
            ),
            spawn=UsdFileCfg(
                usd_path="omni.isaac.robots/Assets/Beakers/glass_beaker_upright.usd",
                rigid_body_props=RigidBodyPropertiesCfg(
                    max_angular_velocity=3.14,
                    max_linear,velocity=10.0,
                ),
            ),
            physics_material=PhysicsMaterialCfg(
                static_friction=0.5,
                dynamic_friction=0.5,
                restitution=0.0,
            ),
            collision_props=CollisionPropertiesCfg(
                enabled=True,
                allowed_collision_filters=["franka", "beaker"],
            ),
        )

        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=FRAME_MARKER_CFG,
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
