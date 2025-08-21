from isaaclab.assets import RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

@configclass
class EventCfg:
    """Configuration for events."""
    pass

@configclass
class FrankaCubeStackEnvCfg:
    def __post_init__(self):
        super().__post_init__()
        # Robot configuration
        self.scene.robot = SceneEntityCfg("franka_panda", spawn="franka_panda", init_state=None)

        # Cube configuration
        self.scene.cube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cubes/cube",
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Assets/Props/Cube/Cube.usd",
                rigid_body_props=RigidBodyPropertiesCfg(),
            ),
            init_state=SceneEntityCfg.InitState(
                pos=(0.0, 0.0, 0.5),
                rot=(0.7071, 0, 0, 0.7071),
                lin_vel=(0, 0, 0),
                ang_vel=(0, 0, 0),
            ),
        )

        # Beaker configuration
        self.scene.beaker = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Beakers/beaker",
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Assets/Props/Beaker/Beaker.usd",
                rigid_body_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
            init_state=SceneEntityCfg.InitState(
                pos=(0.0, 0.5, 0.5),
                rot=(0.7071, 0, 0, 0.7071),
                lin_vel=(0, 0, 0),
                ang_vel=(0, 0, 0),
            ),
        )

        # FrameTransformer configuration
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
                    offset=OffsetCfg(pos=[0.0, 0.0, 0.1034]),
                ),
                FrameTransformerCfg.FrameCfg(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
                    name="tool_rightfinger",
                    offset=OffsetCfg(pos=(0.0, 0.0, 0.046)),
                ),
                FrameTransformer
