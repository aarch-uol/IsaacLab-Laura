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
    reset = EventTerm()
    reset_ee = EventTerm()
    reset_gripper = EventTerm()


class FrankaCubeBeakerEnvCfg(StackEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # Set Franka as the robot
        self.scene.robot = FRANKA_PANDA_CFG
        # Define cube
        self.scene.cube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.7, 0.3, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=RigidBodyPropertiesCfg(),
                semantic_tags=[("class", "cube")],
            ),
        )
        # Define beaker
        self.scene.beaker = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Beaker",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.60, -0.1, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/glass_beaker_upright.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=RigidBodyPropertiesCfg(),
                semantic_tags=[("class", "beaker")],
            ),
        )
        # Set semantics for table and plane
        self.scene.table.semantic_tags = [("class", "table")]
        self.scene.plane.semantic_tags = [("class", "plane")]
        # Set end-effector frame
        self.scene.ee_frame = FrameTransformerCfg(
            name="ee_frame",
            parent_prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                    name="end_effector",
                    offset=OffsetCfg(pos=[0.0, 0.0, 0.1034]),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
                    name="tool_rightfinger",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
                    name="tool_leftfinger",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
                ),
            ],
        )
