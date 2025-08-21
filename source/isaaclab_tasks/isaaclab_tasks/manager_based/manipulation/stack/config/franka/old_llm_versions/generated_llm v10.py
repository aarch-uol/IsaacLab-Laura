from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm, SceneEntityCfg
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
    term = EventTerm()
    term = EventTerm()

class FrankaCubeStackEnvCfg(StackEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.events = EventCfg()
        self.scene.robot = FRANKA_PANDA_CFG.replace(
            {
                "self_collision_enabled": False,
                "joint_default_damping": [100.0] * 14,
                "joint_stiffness": [500.0] * 14,
                "joint_friction": [1.0] * 14,
            }
        )
        self.scene.table.spawn.semantic_tags = [("class", "table")]
        self.scene.plane.semantic_tags = [("class", "ground")]
        self.scene.cube = RigidObjectCfg(
            name="cube",
            file=ISAAC_NUCLEUS_DIR + "/assets/objects/cube.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_body_props=RigidBodyPropertiesCfg(
                density=100.0,
                friction=0.0,
                restitution=0.0,
            ),
            collision_props=RigidBodyPropertiesCfg(
                density=100.0,
                friction=0.0,
                restitution=0.0,
            ),
        )
        self.scene.beaker = RigidObjectCfg(
            name="beaker",
            file=ISAAC_NUCLEUS_DIR + "/assets/objects/beaker.usd",
            scale=(1.0, 1.0, 1.0),
            rigid_body_props=RigidBodyPropertiesCfg(
                density=100.0,
                friction=0.0,
                restitution=0,
            ),
            collision_props=RigidBodyPropertiesCfg(
                density=100.0,
                friction=0.0,
                restitution=0.0,
            ),
        )
        self.scene.ee_frame = FrameTransformerCfg(
            name="ee_frame",
            parent_prim_path="{ENVIRONMENT_PATH}/Robot/panda_hand",
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENVIRONMENT_PATH}/Robot/panda_hand",
                    name="end_effector",
                    offset=OffsetCfg(pos=[0.0, 0.0, 0.1034]),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENVIRONMENT_PATH}/Robot/panda_rightfinger",
                    name="tool_rightfinger",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENVIRONMENT_PATH}/Robot/panda_leftfinger",
                    name="tool_leftfinger",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
                ),
            ],
        )
