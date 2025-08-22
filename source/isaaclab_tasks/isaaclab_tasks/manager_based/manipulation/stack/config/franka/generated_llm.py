from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformerCfg
from isaaclab.sensors.frame_transformer.frame_transformer_cfg import OffsetCfg
from isaaclab.utils import configclass
from isaaclab_tasks.manager_based.manipulation.stack import mdp
from isaaclab_tasks.manager_based.manipulation.stack.lab_env_cfg import StackEnvCfg
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG
from . import glassware_files

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""
    object_dropping1 = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object1")})
    object_dropping2 = DoneTerm(func=mdp.root_height_below_minimum, params={"minimum_height": -0.05, "asset_cfg": SceneEntityCfg("object2")})
    success_term = DoneTerm(func=mdp.objects_stacked, params={"lower_object_cfg": SceneEntityCfg("object3")})
    time_out = DoneTerm(func=mdp.time_out, time_out=True)

@configclass
class ObservationsCfg:
    class PolicyCfg(ObsGroup):
        last_action = ObsTerm(func=mdp.last_action)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)
        object_obs2 = ObsTerm(func=mdp.object_obs, params={"object_cfg": SceneEntityCfg("object1")})
        object_positions_in_world_frame2 = ObsTerm(func=mdp.object_positions_in_world_frame, params={"object_cfg": SceneEntityCfg("object1")})
        object_orientations_in_world_frame2 = ObsTerm(func=mdp.object_orientations_in_world_frame, params={"object_cfg": SceneEntityCfg("object1")})
        ee_frame_pos = ObsTerm(func=mdp.ee_frame_pos)
        ee_frame_quat = ObsTerm(func=mdp.ee_frame_quat)
        gripper_pos = ObsTerm(func=mdp.gripper_pos)

    class SubtasksCfg(ObsGroup):
        # Task 1: Heating a solution
        reach_object_task1 = ObsTerm(func=mdp.reach_object, params={"object_cfg": SceneEntityCfg("object1")})
        object_grasped_task1 = ObsTerm(func=mdp.object_grasped, params={"object_cfg": SceneEntityCfg("object1")})
        is_object_lifted_task1 = ObsTerm(func=mdp.is_object_lifted, params={"object_cfg": SceneEntityCfg("object1")})
        object_reached_midgoal_task1 = ObsTerm(func=mdp.object_reached_midgoal, params={"object_cfg": SceneEntityCfg("object1")})
        reach_object2_task1 = ObsTerm(func=mdp.reach_object2, params={"object_cfg": SceneEntityCfg("object2")})
        object_stacked_task1 = ObsTerm(func=mdp.object_stacked, params={"upper_object_cfg": SceneEntityCfg("object1"), "lower_object_cfg": SceneEntityCfg("object2")})

        # Task 2: Weighing a solution
        reach_object_task2 = ObsTerm(func=mdp.reach_object, params={"object_cfg": SceneEntityCfg("object1")})
        object_grasped_task2 = ObsTerm(func=mdp.object_grasped, params={"object_cfg": SceneEntityCfg("object1")})
        is_object_lifted_task2 = ObsTerm(func=mdp.is_object_lifted, params={"object_cfg": SceneEntityCfg("object1")})
        object_reached_midgoal_task2 = ObsTerm(func=mdp.object_reached_midgoal, params={"object_cfg": SceneEntityCfg("object1")})
        reach_object2_task2 = ObsTerm(func=mdp.reach_object2, params={"object_cfg": SceneEntityCfg("object3")})
        object_stacked_task2 = ObsTerm(func=mdp.object_stacked, params={"upper_object_cfg": SceneEntityCfg("object1"), "lower_object_cfg": SceneEntityCfg("object3")})

    policy = PolicyCfg(enable_corruption=False, concatenate_terms=False)
    subtasks = SubtasksCfg(enable_corruption=False, concatenate_terms=False)

@configclass
class FrankaCubeStackEnvCfg(StackEnvCfg):
    def __post_init__(self):
        # post init of parent
        super().__post_init__()
        glassware = glassware_files.Glassware()
        self.terminations = TerminationsCfg()
        self.observations = ObservationsCfg()

        # Set Franka as robot
        self.scene.robot = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
        self.scene.robot.spawn.semantic_tags = [("class", "robot")]

        # Add semantics to table and ground
        self.scene.table.spawn.semantic_tags = [("class", "table")]
        self.scene.plane.spawn.semantic_tags = [("class", "ground")]

        # Set actions for the specific robot type (franka)
        self.actions.arm_action = mdp.JointPositionActionCfg(asset_name="robot", joint_names=["panda_joint.*"], scale=0.5, use_default_offset=True)
        self.actions.gripper_action = mdp.BinaryJointPositionActionCfg(asset_name="robot", joint_names=["panda_finger.*"], open_command_expr={"panda_finger_.*": 0.04}, close_command_expr={"panda_finger_.*": 0.0})
        self.commands.object_pose.body_name = "panda_hand"

        # Spawn Glassware
        self.scene.object1 = glassware.beaker  # Main object for heating and weighing

        # Spawn Lab Equipment
        self.scene.object2 = glassware.hot_plate  # Hot plate for Task 1: Heating a solution
        self.scene.object3 = glassware.electric_balance  # Electric balance for Task 2: Weighing a solution

        # Frame Transformations
        marker_cfg = FRAME_MARKER_CFG.copy()
        marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
        marker_cfg.prim_path = "/Visuals/FrameTransformer"
        self.scene.ee_frame = FrameTransformerCfg(
            prim_path="{ENV_REGEX_NS}/Robot/panda_link0",
            debug_vis=False,
            visualizer_cfg=marker_cfg,
            target_frames=[
                FrameTransformerCfg.FrameCfg(prim_path="{ENV_REGEX_NS}/Robot/panda_hand", name="end_effector", offset=OffsetCfg(pos=[0.0, 0.0, 0.1034])),
                FrameTransformerCfg.FrameCfg(prim_path="{ENV_REGEX_NS}/Robot/panda_rightfinger", name="tool_rightfinger", offset=OffsetCfg(pos=(0.0, 0.0, 0.046))),
                FrameTransformerCfg.FrameCfg(prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger", name="tool_leftfinger", offset=OffsetCfg(pos=(0.0, 0.0, 0.046))),
            ],
        )
