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

##
# Pre-defined configs
##
from isaaclab.markers.config import FRAME_MARKER_CFG  # isort: skip
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG  # isort: skip


@configclass
class EventCfg:
    """Configuration for events."""

    init_franka_arm_pose = EventTerm(
        func=franka_stack_events.set_default_joint_pose,
        mode="startup",
        params={
            "default_pose": [0.0444, -0.1894, -0.1107, -2.5148, 0.0046, 0.0, 0.0],
        },
    )

    randomize_end_effector = EventTerm(
        func=franka_stack_events.randomize_end_effector,
        mode="loop",
        params={"min": 0.0, "max": 0.1},
    )


@configclass
class FrankaCubeStackEnvCfg(StackEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        # Set up the robot
        self.scene.robot = FRANKA_PANDA_CFG
        # Add semantics to the table and ground
        self.scene.table = SceneEntityCfg("Table", prim_path="/World/ground", category="ignore")
        self.scene.ground = SceneEntityCfg("Ground", prim_path="/World/ground", category="ignore")
        # Set up the robot actions
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=FRAME_MARKER_CFG,
            target_frames=[
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                    name="end_effector",
                    offset=OffsetCfg(pos=[0.0, 0.0, 0.1034]),
                ),
                FrameTransformer,  # This line is a placeholder for the actual FrameTransformerCfg
            ],
        )
        # Configure the cube
        self.scene.cube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cubes/cube",
            name="cube",
            translation=(0.5, 0.0, 0.5),
            rotation=(0.0, 0.0, 0.0, 1.0),
            scale=(0.05, 0.05, 0.05),
        )
        # Configure the beaker
        self.scene.beaker = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cubes/beaker",
            name="beaker",
            translation=(0.0, 0.5, 0.5),
            rotation=(0.0, 0.0, 0.0, 1.0),
            scale=(0.05, 0.05, 0.05),
        )
