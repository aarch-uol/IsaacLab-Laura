from isaaclab.assets import RigidObjectCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


class Glassware:
    def __init__(self):
        # Rigid body properties of each cube
        cube_properties = RigidBodyPropertiesCfg(
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_angular_velocity=1000.0,
            max_linear_velocity=1000.0,
            max_depenetration_velocity=5.0,
            disable_gravity=False,
        )

        # Spawn a sample vial
        self.sample_vial = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Sample_vial",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.0, 0.0203], rot=[0, 1, 1, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_glass.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=cube_properties,
                semantic_tags=[("class", "sample_vial")],
            ),
        )
        # Spawn a cube
        self.cube = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Cube",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0.0, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/red_block.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=cube_properties,
                semantic_tags=[("class", "cube")],
            ),
        )
        # Spawn a conical vial
        self.conical_flask = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Conical_flask",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.60, -0.1, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/conical_flask_100ml_glass_new.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=cube_properties,
                semantic_tags=[("class", "conical_flask")],
            ),
        )
        # Spawn a beaker
        self.beaker = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Beaker",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.60, -0.1, 0.0203], rot=[1, 0, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/glass_beaker_upright.usd",
                scale=(1.0, 1.0, 1.0),
                rigid_props=cube_properties,
                semantic_tags=[("class", "beaker")],
            ),
        )
        # Spawn a round bottom flask
        self.round_bottom_flask = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Round_bottom_flask",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.80, -0.1, 0.0203], rot=[0.707, 0.707, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/glass_round_bot.usd",
                scale=(0.01, 0.01, 0.01),
                rigid_props=cube_properties,
                semantic_tags=[("class", "round_bottom_flask")],
            ),
        )
        # Spawn a hot plate
        self.hot_plate = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Hot_plate",
            init_state=RigidObjectCfg.InitialStateCfg(pos=[0.7, 0.3, 0.0203], rot=[0.707, 0.707, 0, 0]),
            spawn=UsdFileCfg(
                usd_path=f"/workspace/isaaclab/source/isaaclab_assets/data/Props/lab_equipment/hot_plate.usd",
                scale=(0.03, 0.03, 0.03),
                rigid_props=cube_properties,
                semantic_tags=[("class", "hot_plate")],
            ),
        )