from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot
from isaaclab.envs import robot

class EventCfg:
    pass

class FrankaCubeStackEnvCfg(robot.RobotEnvCfg):
    def __init__(self, *args, **kwargs):
        super().__post_init__()
        self.scene.robot = robot.FrankaPandaCfg()
        self.scene.cube = robot.CubeCfg(
            prim_path="{ENV_REGEX_NS}/Cuboid",
            spawn_position=(0.0, 0.0, 0.5),
            spawn_orientation=(0.0, 0.0, 0.0, 1.0),
            scale=(1.0, 1.0, 1.0),
            rigid_props=robot.RigidBodyPropertiesCfg(),
            visual_material=robot.PhysXMaterialCfg(static_friction=0.5, dynamic_friction=0.5, restitution=0.0),
            collision_material=robot.PhysXMaterialCfg(static_friction=0.5, dynamic_friction=0.5, restitution=0.0),
            name="cube",
            category=1
        )
        self.scene.beaker = robot.BeakerCfg(
            prim_path="{ENV_REGEX_NS}/Beaker",
            spawn_position=(0.0, 0.5, 0.5),
            spawn_orientation=(0.0, 0.0, 0.0, 1.0),
            scale=(1.0, 1.0, 1.0),
            rigid_props=robot.RigidBodyPropertiesCfg(
                solver_iterations=10,
                contact_surface_offset=0.01
            ),
            visual_material=robot.PhysXMaterialCfg(static_friction=0.5, dynamic_friction=0.5, restitution=0.0),
            collision_material=robot.PhysXMaterialCfg(static_friction=0.5, dynamic_friction=0.5, restitution=0.0),
            name="beaker",
            category=1
        )
        self.scene.ee_frame = robot.FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=robot.VisualizerCfg(
                prim_path="/Visuals/FrameTransformer",
                draw_prefix="",
                visuals=[
                    robot.VisualizerCfg.Visual(
                        name="end_effector",
                        prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                        offset=robot.OffsetCfg(pos=(0.0, 0.0, 0.1034)),
                        color=(0.0, 1.0, 0.0, 1.0)
                    ),
                    robot.VisualizerCfg.Visual(
                        name="tool_rightfinger",
                        prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
                        offset=robot.OffsetCfg(pos=(0.0, 0.0, 0.046)),
                        color=(1.0, 0.0, 0.0, 1.0)
                    ),
                    robot.VisualizerCfg.Visual(
                        name="tool_leftfinger",
                        prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
                        offset=robot.OffsetCfg(pos=(0.0, 0.0, 0.046)),
                        color=(0.0, 0.0, 1.0, 1.0)
                    )
                ]
            ),
            target_frames=[
                robot.FrameTransformerCfg.Frame(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_hand",
                    name="end_effector",
                    offset=robot.OffsetCfg(pos=(0.0, 0.0, 0.1034))
                ),
                robot.FrameTransformerCfg.Frame(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger",
                    name="tool_rightfinger",
                    offset=robot.OffsetCfg(pos=(0.0, 0.0, 0.046))
                ),
                robot.FrameTransformerCfg.Frame(
                    prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
                    name="tool_leftfinger",
                    offset=robot.OffsetCfg(pos=(0.0, 0.0, 0.046))
                )
            ]
        )
