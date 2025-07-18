#region Header
# IsaacLab Task Script
# --------------------------------------------------------------------------------
# File: 
# Author: Harry WF Cheung (University of Liverpool)
# Date: 16/06/2025
# Description: 
# --------------------------------------------------------------------------------
# License: BSD-3-Clause (© 2022-2025 Isaac Lab Project Developers)
# --------------------------------------------------------------------------------

# --------------------------------------------------------------------------------
# region Imports

from isaaclab.utils import configclass
from isaaclab.assets import RigidObjectCfg
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg
from isaaclab_tasks.manager_based.manipulation.stack.stack_env_cfg import (
    ObservationsCfg, TerminationsCfg, 
)
from isaaclab_tasks.manager_based.manipulation.stack.config.franka.stack_joint_pos_env_cfg import EventCfg 
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from chills.envs.franka_env import FrankaIkEnvCfg

# --------------------------------------------------------------------------------
# region Task: Cubes

@configclass
class CubeObjCfg(RigidObjectCfg):
    """Config class for colored cube objects using Nucleus props."""
    colour: str = "blue"
    num: int = 1
    pos: list = (0.4, 0.0, 0.0203)

    def __post_init__(self):
        super().__post_init__()
        self.prim_path = f"{{ENV_REGEX_NS}}/Cube_{self.num}"
        self.init_state = RigidObjectCfg.InitialStateCfg(self.pos, rot=[1, 0, 0, 0])
        self.spawn = UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/{self.colour}_block.usd",
            scale=(1.0, 1.0, 1.0),
            semantic_tags=[("class", f"Cube_{self.num}")],
            rigid_props=RigidBodyPropertiesCfg(
                solver_position_iteration_count=16,
                solver_velocity_iteration_count=1,
                max_angular_velocity=1000.0,
                max_linear_velocity=1000.0,
                max_depenetration_velocity=5.0,
                disable_gravity=False,
            ),
        )


# --------------------------------------------------------------------------------
# region Task: Base

@configclass
class StackEnvCfg(FrankaIkEnvCfg):
    """Configuration for the stacking environment."""

    observations: ObservationsCfg = ObservationsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    commands = None
    rewards = None
    events = EventCfg()
    curriculum = None

    def __post_init__(self):
        """Post initialization."""
        super().__post_init__()
        self.decimation = 5

        self.scene.cube_1 = CubeObjCfg.replace(colour="blue", num=1, pos=[0.4, 0.0, 0.0203])
        self.scene.cube_2 = CubeObjCfg.replace(colour="red", num=2, pos=[0.55, 0.05, 0.0203])
        self.scene.cube_3 = CubeObjCfg.replace(colour="green", num=3, pos=[0.60, -0.1, 0.0203])

