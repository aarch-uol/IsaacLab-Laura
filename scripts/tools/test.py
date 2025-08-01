import argparse
import os
from isaaclab.app import AppLauncher

# add argparse arguments
# create argparser
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
args_cli.experience = "isaacsim.exp.full.kit"  # Set the experience here
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


"""ROS2 stuff"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Replace with your desired message type

"""Rest everything follows."""
import math
import torch
import math
import os
from isaaclab.envs import ManagerBasedRLEnv
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs import mdp
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

# Create custom cfg file for articulated quadruped spawning
WHEELED_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=robot_usd_path,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=100.0,
            max_angular_velocity=50.0,
            enable_gyroscopic_forces=True,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.5),
        joint_pos={"joint_wheel_left": 0.0, "joint_wheel_right": 0.0},
    ),
    actuators={
        "left_wheel_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_wheel_left"],
            effort_limit=300.0,
            velocity_limit=10.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "right_wheel_actuator": ImplicitActuatorCfg(
            joint_names_expr=["joint_wheel_right"],
            effort_limit=300.0,
            velocity_limit=10.0,
            stiffness=0.0,
            damping=10.0,
        ),
    },
)


# Scene cfg
@configclass
class NirisWheeledSceneCfg(InteractiveSceneCfg):
    """Configuration for scene."""

    # dome light
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.9, 0.9, 0.9), intensity=500.0),
    )

    # distant lights
    distant_light = AssetBaseCfg(
        prim_path="/World/DistantLight",
        spawn=sim_utils.DistantLightCfg(color=(0.9, 0.9, 0.9), intensity=2500.0),
        init_state=AssetBaseCfg.InitialStateCfg(rot=(0.738, 0.477, 0.477, 0.0)),
    )

    # Load custom environment USD file
    custom_env = AssetBaseCfg(
        prim_path="/World/CustomEnv",
        spawn=sim_utils.UsdFileCfg(usd_path=env_usd_path),
    )

    # quadruped robot
    robot: ArticulationCfg = WHEELED_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")


# Actions cfg
@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    # velocities of back wheels
    joint_velocities = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["joint_wheel_left", "joint_wheel_right"],
        scale=1.0,
    )
    """ 
    # positions of front thighs
    joint_positions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["robot1_front_left_thigh_joint", "robot1_front_right_thigh_joint"],
        scale=1.0,
    )
"""

# Observations cfg
@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


# Event cfg
@configclass
class EventCfg:
    """Configuration for events."""

    reset_scene = EventTerm(func=mdp.reset_scene_to_default, mode="reset")


# Rewards cfg
@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    # (3) Primary task: keep robot upright
    pole_pos = RewTerm(
        func=mdp.base_height_l2,
        weight=1.0,
        params={"asset_cfg": SceneEntityCfg("robot"), "target_height": 0.828},
    )


# Terminations cfg
@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) The robot fell on the ground
    """
    robot_on_the_ground = DoneTerm(
        func=mdp.root_height_below_minimum,
        params={"asset_cfg": SceneEntityCfg("robot"), "minimum_height": 0.4},
    )
    """
    robot_on_the_ground = DoneTerm(
        func=mdp.bad_orientation,
        params={"asset_cfg": SceneEntityCfg("robot"), "limit_angle": math.pi / 3},
    )


@configclass
class CommandsCfg:
    """Command terms for the MDP."""

    # no commands for this MDP
    null = mdp.NullCommandCfg()


@configclass
class CurriculumCfg:
    """Configuration for the curriculum."""

    pass


@configclass
class WheeldQudrupedEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the wheeled quadruped environment."""

    # Scene settings
    scene: NirisWheeledSceneCfg = NirisWheeledSceneCfg(num_envs=4096, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    curriculum: CurriculumCfg = CurriculumCfg()
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    # No command generator
    commands: CommandsCfg = CommandsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        # simulation settings
        self.sim.dt = 0.005  # simulation timestep -> 200 Hz physics
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.render_interval = self.decimation


def main():
    """Main function."""
    # create environment configuration
    print("Main")
    env_cfg = WheeldQudrupedEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    # setup RL environment
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # simulate physics
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # sample random actions
            joint_vel = torch.randn_like(env.action_manager.action)
            # step the environment
            obs, rew, terminated, truncated, info = env.step(joint_vel)
            # print current orientation of pole
            print("[Env 0]: Pole joint: ", obs["policy"][0][1].item())
            # update counter
            count += 1

    # close the environment
    env.close()

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()