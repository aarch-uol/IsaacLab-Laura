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
    init_franka_arm_pose: EventTerm = EventTerm(
        command="init_franka_arm_pose",
        action="set_joint_positions",
        params={"joint_positions": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571]},
    )
    randomize_franka_joint_state: EventTerm = EventTerm(
        command="randomize_franka_joint_state",
        action="randomize_joint_positions",
        params={"min_joint_positions": [-2.0, -2.0, -2.0, -2.0, -2.0, -2.0],
                "max_joint_positions": [2.0, 2.0, 2.0, 2.0, 2.0, 2.0]},
    )


@configclass
class FrankaCubeStackEnvCfg(StackEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # Set Franka as robot
        self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.spawn.semantic_tags = [("class", "robot")]
        # Set table and plane semantics
        self.scene.table.spawn.semantic_tags = [("class", "table")]
        self.scene.plane.spawn.semantic_tags = [("class", "floor")]
        # Define actions
        self.actions.arm = mdp.JointAction(
            name="arm",
            joints=["panda_joint1", "panda_joint2", "panda_joint3",
                    "panda_joint4", "panda_joint5", "panda_joint6"],
        )
        # Define cube
        self.scene.cube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube",
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.7, 0.3, 0.0203], rot=[1.0, 0.0, 0.0, 0.0]),
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
            init_state=RigidObjectCfg.InitialStateCfg(
                pos=[0.60, -0.1, 0.0203], rot=[1.0, 0.0, 0.0, 0.0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/glass_beaker_upright.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=RigidBodyPropertiesCfg(),
                semantic_tags=[("class", "beaker")],
            ),
        )
        # Define end-effector frame transformer
        self.scene.end_effector = FrameTransformerCfg(
            name="end_effector",
            parent_prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
            offset=OffsetCfg(pos=[0.0, 0.0, 0.1034]),
        )
        self.scene.tool_rightfinger = FrameTransformerCfg(
            name="tool_rightfinger",
            parent_prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
            offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
        )
        self.scene.tool_leftfinger = FrameTransformerCfg(
            name="tool_leftfinger",
            parent_prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
            offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
        )
