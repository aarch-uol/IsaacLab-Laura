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
from isaaclab_tasks.manager_based.manipulation.stack.mdp import franka_hand
from isaaclab_tasks.manager_based.manipulation.stack.mdp import franka_leftfinger
from isaaclab_tasks.manager_based.manipulation.stack.mdp import franka_rightfinger

@configclass
class FrankaCubeStackEnvCfg:
    def __post_init__(self):
        # Define robot
        self.scene.robot = SceneEntityCfg("Franka", prim_path="/World/Franka", spawn="franka_description/robots/franka.urdf", rigid_body_props={"friction": 1.0, "restitution": 0.0})
        
        # Define cube
        self.scene.cube = RigidObjectCfg(
            prim_path="/World/Objects/cube",
            spawn="cube",
            rigid_body_props={"friction": 1.0, "restitution": 0.0},
            visual_material=MaterialCfg(pbr_props={"baseColor": (0.8, 0.8, 0.8, 1.0)})
        )
        
        # Define beaker
        self.scene.beaker = RigidObjectCfg(
            prim_path="/World/Objects/beaker",
            spawn="beaker",
            rigid_body_props={"friction": 1.0, "restitution": 0.0},
            visual_material=MaterialCfg(pbr_props={"baseColor": (0.3, 0.3, 0.8, 1.0)})
        )
        
        # Define FrameTransformers
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
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
