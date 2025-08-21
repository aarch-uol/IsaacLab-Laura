import numpy as np
import omni.isaac.core
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import RigidObject
from omni.isaac.core.simulation import Simulation
from omni.isaac.core.utils.types import ArticulationCfg, AssetBaseCfg

def design_scene():
    # Create Franka robot
    franka_cfg = ArticulationCfg(
        asset_file="Franka/Isaac/FRANKA_Panda/FRANKA_Panda.usd",
        spawn_config=AssetBaseCfg(
            position=np.array([0, 0, 0.7]),
            orientation=np.array([0, 0, 0, 1])
        )
    )
    franka = Articulation(franka_cfg)
    # Create blue cube
    cube_cfg = RigidObjectCfg(
        prim_path="/World/BlueCube",
        spawn_config=AssetBaseCfg(
            position=np.array([0.5, 0, 0.2]),
            orientation=np.array([0, 0, 0, 1]),
            color=np.array([0, 0, 1, 1])  # Blue
        )
    )
    cube = RigidObject(cube_cfg)
    return cube

def main():
    # Initialize simulation
    sim = Simulation()
    # Design the scene
    cube = design_scene()
    # Run simulation loop
    while sim.is_running():
        # Check if cube is on the table
        cube_pos = cube.get_position()
        if cube_pos[2] < 0.1:  # Assuming table is at z=0
            print("Task complete")
            break
        # Step simulation
        sim.step()

if __name__ == "__main__":
    main()
