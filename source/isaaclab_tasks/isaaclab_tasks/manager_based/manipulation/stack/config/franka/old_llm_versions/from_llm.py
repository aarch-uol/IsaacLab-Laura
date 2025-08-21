import argparse

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Frozen scene of a user input object on a table.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import isaacsim.core.utils.prims as prim_utils
import torch

import isaaclab.sim as sim_utils
import isaaclab.utils.math as math_utils
from isaaclab.assets import RigidObject, RigidObjectCfg, AssetBaseCfg, Articulation
from isaaclab.sim.schemas.schemas_cfg import RigidBodyPropertiesCfg
from isaaclab.sim.spawners.from_files.from_files_cfg import UsdFileCfg, GroundPlaneCfg
from isaaclab.sim import SimulationContext

from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils import configclass

FILE = '/workspace/isaaclab/source/isaaclab_assets/data/Props'
TABLE_FILE = 'Props/Mounts/SeattleLabTable/table_instanceable.usd'
ENV = '/World/env/env_0'

def design_scene():
    # Table
    table_path = f'{ISAAC_NUCLEUS_DIR}/{TABLE_FILE}'
    table = AssetBaseCfg(
        prim_path=f"{ENV}/Table",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0.5, 0, 0], rot=[0.707, 0, 0, 0.707]),
        spawn=UsdFileCfg(usd_path=table_path),
    )
    # plane
    plane = AssetBaseCfg(
        prim_path="/World/GroundPlane",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[0, 0, -1.05]),
        spawn=GroundPlaneCfg(),
    )
    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

    # Set Franka as robot
    robot_cfg = FRANKA_PANDA_CFG.replace(prim_path=f"{ENV}/Robot")
    robot = Articulation(robot_cfg)

    ## Add an object
    object_path = f'{FILE}/glassware/beaker.usd'
    sim_object_cfg = RigidObjectCfg(
        prim_path=f'{ENV}/Object',
        init_state=RigidObjectCfg.InitialStateCfg(pos=[0.5, 0, 0.055], rot=[1, 0, 0, 0]),
        spawn=UsdFileCfg(
            usd_path=object_path,
            scale=(1, 1, 1),
            rigid_props=RigidBodyPropertiesCfg(),
        ),
    )

    RigidObject(sim_object_cfg)
    RigidObject(light)
    RigidObject(table)
    RigidObject(plane)
    return robot

def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.0, 0.0, 2.5], [-0.5, 0.0, 0.5])

    # Design scene by adding assets to it
    robot = design_scene()

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
