from isaaclab.sim import MdlFileCfg
from isaaclab.assets import RigidObjectCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg, CollisionPropertiesCfg, MassPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab.sim.spawners.wrappers import MultiUsdFileCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
#from source.isaaclab.isaaclab.sim.spawners.shapes.shapes_cfg import SphereCfg
from isaaclab.sim.spawners.shapes.shapes_cfg import SphereCfg
from isaaclab.sim import (
    UsdFileCfg, SphereCfg, RigidBodyPropertiesCfg, CollisionPropertiesCfg,
    RigidBodyMaterialCfg, PreviewSurfaceCfg, GlassMdlCfg
)
@configclass
class ChemistryGlassware:
        cube_properties = RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        )
        fixed_properties = RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
            kinematic_enabled=True,
        )

        mass_props = MassPropertiesCfg(
            mass=10
        )

        glass = GlassMdlCfg(glass_color=(1.0, 1.0, 1.0), frosting_roughness=0.1, thin_walled=True, glass_ior=1.47)
        
        plastic = MdlFileCfg(mdl_path="{NVIDIA_NUCLEUS_DIR}/Materials/2023_1/Base/Plastics/Plastic_ABS.mdl")
        # Collision properties with convex hull approximation for dynamic bodies
        collision_properties = CollisionPropertiesCfg(
            collision_enabled=True,
        )

        def flask(self,pos=[0.4, 0.35, 0.0203],rot=[0, 0, 1, 0], name="Flask", scale=(1,1,1))-> RigidObjectCfg:
           return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/isaaclab_assets/isaaclab_assets/glassware/conical_flask/conical_flask.usd",
                    scale=scale,
                    rigid_props=self.cube_properties,
                    collision_props=self.collision_properties,
                    visible=True,
                    copy_from_source = False,
                    visual_material=self.glass,
                    semantic_tags=[("class", "Flask")],
                ),
            ) 

        def vial(self,pos=[0.4, 0.35, 0.0203],rot=[0, 0, 1, 0], name="Vial")-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/isaaclab_assets/isaaclab_assets/glassware/sample_vial/sample_vial.usd",
                    scale=(1, 1, 1),
                    rigid_props=self.cube_properties,
                    collision_props=self.collision_properties,
                    visible=True,
                    copy_from_source = False,
                    visual_material=self.glass,
                    semantic_tags=[("class", name)],
                ),
            ) 

        def beaker(self,pos=[0.4, 0.35, 0.0203],rot=[0, 0, 1, 0], name="Beaker")-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name,
                init_state=RigidObjectCfg.InitialStateCfg(pos = pos,rot=rot ),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/isaaclab_assets/isaaclab_assets/glassware/beaker/beaker.usd",
                    scale=(0.5, 0.5, 0.5),
                    rigid_props=self.cube_properties,
                    collision_props=self.collision_properties,
                    visual_material=self.glass,
                    semantic_tags=[("class", name)],
                ),
            )

        def stirplate(self,pos=[0.46, -0.3, 0.0],rot=[0.5, 0.5, 0.5, 0.5], name="Stirplate")-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name ,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/isaaclab_assets/isaaclab_assets/glassware/ikaplate/ikaplate.usd",
                    scale=(1.0, 1.0, 1.0),
                    rigid_props=self.cube_properties,
                    collision_props=self.collision_properties,
                    semantic_tags=[("class", name)],
                    visual_material=self.plastic
                ),
            ) 

        def hotplate(self,pos=[0.46, -0.3, 0.0],rot=[0.5, 0.5, 0.5, 0.5], name="Hotplate")-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name ,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/chills/chills/assets/hotplate/hotplate.usd",
                    scale=(1.0, 1.0, 1.0),
                    rigid_props=self.cube_properties,
                    collision_props=self.collision_properties,
                    semantic_tags=[("class", name)],
                ),
            ) 

        def scale(self,pos=[0.46, -0.3, 0.0],rot=[1, 0, 0, 0], name="Scale")-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name ,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/chills/chills/assets/scale/scale.usd",
                    scale=(0.2, 0.2, 0.2),
                    rigid_props=self.cube_properties,
                    collision_props=self.collision_properties,
                    semantic_tags=[("class", name)],
                ),
            ) 

        def clamp_stand(self,pos=[0.2, 0.3, 0.05],rot=[0.5, 0.5, 0.5, 0.5], name="Clampstand")-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name ,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/chills/chills/assets/full_clamp_stand/full_clamp_stand.usd",
                    scale=(1.0, 1.0, 1.0),
                    rigid_props=self.cube_properties,
                    collision_props=self.collision_properties,
                    semantic_tags=[("class", name)],
                ),
            )

        def capped_vial(self,pos=[0.2, 0.3, 0.05],rot=[1,0,0,0], scale=1.0, name="capped_vial")-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name ,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/isaaclab_assets/isaaclab_assets/glassware/vial_with_cap/vial_with_cap_non_ins.usd",
                    scale=(scale, scale, scale),
                    rigid_props=self.cube_properties,
                    collision_props=self.collision_properties,
                    visual_material=self.glass,
                    semantic_tags=[("class", name)],
                ),
            )

        def line_vial_rack(self,pos=[0.2, 0.3, 0.05],rot=[1,0,0,0], name="vial_rack",scale=1.0)-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name ,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=UsdFileCfg(
                    usd_path="/workspace/isaaclab/source/isaaclab_assets/isaaclab_assets/glassware/vial_rack/vial_rack_non_ins.usd",
                    scale=(scale, scale, 1.0),
                    rigid_props=self.cube_properties,
                    mass_props=self.mass_props,
                    #collision_props=self.collision_properties,
                    visual_material=self.plastic,
                    semantic_tags=[("class", name)],
                ),
            )  
        # def particles(self,radius=0.002, color=(0.1,0.4,0.9), static_mu=0.05, dynamic_mu=0.05,
        #       restitution=0.0, contact_offset_scale=0.3, rest_offset_scale=-0.02,
        #       lin_damp=3.0, ang_damp=3.0):
        #     return SphereCfg(
        #         radius=radius,
        #         visual_material=PreviewSurfaceCfg(diffuse_color=color),
        #         physics_material=RigidBodyMaterialCfg(
        #             static_friction=static_mu, dynamic_friction=dynamic_mu, restitution=restitution,
        #             friction_combine_mode="min", restitution_combine_mode="min"
        #         ),
        #         collision_props=CollisionPropertiesCfg(
        #             contact_offset=contact_offset_scale*radius, rest_offset=rest_offset_scale*radius
        #         ),
        #         rigid_props=RigidBodyPropertiesCfg(
        #             disable_gravity=False, linear_damping=lin_damp, angular_damping=ang_damp,
        #             solver_position_iteration_count=32, solver_velocity_iteration_count=6,
        #             max_depenetration_velocity=0.5, max_linear_velocity=0.1, max_angular_velocity=0.1,
        #             sleep_threshold=0.05,
        #         ),
        #     )
        
        def random_object(self,pos=[0.65, 0.3, 0.05],rot=[0, 0, 1, 0], name="random")-> RigidObjectCfg:
            return RigidObjectCfg(
                prim_path="{ENV_REGEX_NS}/" + name ,
                init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
                spawn=MultiUsdFileCfg(
                    usd_path=["/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/beaker.usd", "/workspace/isaaclab/source/isaaclab_assets/data/Props/glassware/sample_vial_20ml.usd"],
                    scale=(0.8, 0.8, 0.8),
                    rigid_props=self.cube_properties,
                    semantic_tags=[("class", name)],
                    random_choice = True
                ),
            )

        def cube_obs(self,pos=[0.65, 0.3, 0.05],rot=[0, 0, 1, 0], name="cube_obs") -> RigidObjectCfg:
            return RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/" + name,
            init_state=RigidObjectCfg.InitialStateCfg(pos=pos,rot=rot),
            spawn=UsdFileCfg(
                usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
                scale=(1.2, 1.2, 1.2),
                rigid_props=RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16,
                    solver_velocity_iteration_count=1,
                    max_angular_velocity=1000.0,
                    max_linear_velocity=1000.0,
                    max_depenetration_velocity=5.0,
                    disable_gravity=False,
                ),
            ),
        )        