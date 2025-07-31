<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e (Fresh start, no history)
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> e9462be776417c5794982ad017c44c19fac790a2
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Launch Isaac Sim Simulator first."""

from isaaclab.app import AppLauncher

# launch omniverse app
simulation_app = AppLauncher(headless=True).app

"""Rest everything follows."""

<<<<<<< HEAD
<<<<<<< HEAD
import torch

=======
>>>>>>> abfba5273e (Fresh start, no history)
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
import pytest

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.sim import build_simulation_context
<<<<<<< HEAD
<<<<<<< HEAD
=======
from isaaclab.terrains import TerrainImporterCfg
>>>>>>> abfba5273e (Fresh start, no history)
=======
from isaaclab.terrains import TerrainImporterCfg
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR


@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Example scene configuration."""

<<<<<<< HEAD
<<<<<<< HEAD
    # articulation
    robot = ArticulationCfg(
        prim_path="/World/envs/env_.*/Robot",
=======
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
    # terrain - flat terrain plane
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
    )

    # articulation
    robot = ArticulationCfg(
        prim_path="/World/Robot",
<<<<<<< HEAD
>>>>>>> abfba5273e (Fresh start, no history)
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
        spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Simple/revolute_articulation.usd"),
        actuators={
            "joint": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=100.0, damping=1.0),
        },
    )
    # rigid object
    rigid_obj = RigidObjectCfg(
<<<<<<< HEAD
<<<<<<< HEAD
        prim_path="/World/envs/env_.*/RigidObj",
=======
        prim_path="/World/RigidObj",
>>>>>>> abfba5273e (Fresh start, no history)
=======
        prim_path="/World/RigidObj",
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
        spawn=sim_utils.CuboidCfg(
            size=(0.5, 0.5, 0.5),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
            ),
            collision_props=sim_utils.CollisionPropertiesCfg(
                collision_enabled=True,
            ),
        ),
    )

<<<<<<< HEAD
<<<<<<< HEAD

@pytest.fixture
def setup_scene(request):
    """Create simulation context with the specified device."""
    device = request.getfixturevalue("device")
    with build_simulation_context(device=device, auto_add_lighting=True, add_ground_plane=True) as sim:
        sim._app_control_on_stop_handle = None

        def make_scene(num_envs: int, env_spacing: float = 1.0):
            scene_cfg = MySceneCfg(num_envs=num_envs, env_spacing=env_spacing)
            return scene_cfg

        yield make_scene, sim
    sim.stop()
    sim.clear()
    sim.clear_all_callbacks()
    sim.clear_instance()
=======
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
    # sensor
    sensor = ContactSensorCfg(
        prim_path="/World/Robot",
    )
    # extras - light
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(),
    )


@pytest.fixture(scope="module")
def setup_scene():
    """Fixture to set up scene parameters."""
    sim_dt = 0.001
    scene_cfg = MySceneCfg(num_envs=1, env_spacing=1)
    return sim_dt, scene_cfg
<<<<<<< HEAD
>>>>>>> abfba5273e (Fresh start, no history)
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5


@pytest.mark.parametrize("device", ["cuda:0", "cpu"])
def test_scene_entity_isolation(device, setup_scene):
    """Tests that multiple instances of InteractiveScene do not share any data.

    In this test, two InteractiveScene instances are created in a loop and added to a list.
    The scene at index 0 of the list will have all of its entities cleared manually, and
    the test compares that the data held in the scene at index 1 remained intact.
    """
<<<<<<< HEAD
<<<<<<< HEAD
    make_scene, sim = setup_scene
    scene_cfg = make_scene(num_envs=1)
    # set additional light to test 'extras' attribute of the scene
    setattr(
        scene_cfg,
        "light",
        AssetBaseCfg(
            prim_path="/World/light",
            spawn=sim_utils.DistantLightCfg(),
        ),
    )
    # set additional sensor to test 'sensors' attribute of the scene
    setattr(scene_cfg, "sensor", ContactSensorCfg(prim_path="/World/envs/env_.*/Robot"))

    scene_list = []
    # create two InteractiveScene instances
    for _ in range(2):
        with build_simulation_context(device=device, dt=sim.get_physics_dt()) as _:
=======
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
    sim_dt, scene_cfg = setup_scene
    scene_list = []
    # create two InteractiveScene instances
    for _ in range(2):
        with build_simulation_context(device=device, dt=sim_dt) as _:
<<<<<<< HEAD
>>>>>>> abfba5273e (Fresh start, no history)
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
            scene = InteractiveScene(scene_cfg)
            scene_list.append(scene)
    scene_0 = scene_list[0]
    scene_1 = scene_list[1]
    # clear entities for scene_0 - this should not affect any data in scene_1
    scene_0.articulations.clear()
    scene_0.rigid_objects.clear()
    scene_0.sensors.clear()
    scene_0.extras.clear()
    # check that scene_0 and scene_1 do not share entity data via dictionary comparison
    assert scene_0.articulations == dict()
    assert scene_0.articulations != scene_1.articulations
    assert scene_0.rigid_objects == dict()
    assert scene_0.rigid_objects != scene_1.rigid_objects
    assert scene_0.sensors == dict()
    assert scene_0.sensors != scene_1.sensors
    assert scene_0.extras == dict()
    assert scene_0.extras != scene_1.extras
<<<<<<< HEAD
<<<<<<< HEAD


@pytest.mark.parametrize("device", ["cuda:0", "cpu"])
def test_relative_flag(device, setup_scene):
    make_scene, sim = setup_scene
    scene_cfg = make_scene(num_envs=4)
    scene = InteractiveScene(scene_cfg)
    sim.reset()

    # test relative == False produces different result than relative == True
    assert_state_different(scene.get_state(is_relative=False), scene.get_state(is_relative=True))

    # test is relative == False
    prev_state = scene.get_state(is_relative=False)
    scene["robot"].write_joint_state_to_sim(
        position=torch.rand_like(scene["robot"].data.joint_pos), velocity=torch.rand_like(scene["robot"].data.joint_pos)
    )
    next_state = scene.get_state(is_relative=False)
    assert_state_different(prev_state, next_state)
    scene.reset_to(prev_state, is_relative=False)
    assert_state_equal(prev_state, scene.get_state(is_relative=False))

    # test is relative == True
    prev_state = scene.get_state(is_relative=True)
    scene["robot"].write_joint_state_to_sim(
        position=torch.rand_like(scene["robot"].data.joint_pos), velocity=torch.rand_like(scene["robot"].data.joint_pos)
    )
    next_state = scene.get_state(is_relative=True)
    assert_state_different(prev_state, next_state)
    scene.reset_to(prev_state, is_relative=True)
    assert_state_equal(prev_state, scene.get_state(is_relative=True))


@pytest.mark.parametrize("device", ["cuda:0", "cpu"])
def test_reset_to_env_ids_input_types(device, setup_scene):
    make_scene, sim = setup_scene
    scene_cfg = make_scene(num_envs=4)
    scene = InteractiveScene(scene_cfg)
    sim.reset()

    # test env_ids = None
    prev_state = scene.get_state()
    scene["robot"].write_joint_state_to_sim(
        position=torch.rand_like(scene["robot"].data.joint_pos), velocity=torch.rand_like(scene["robot"].data.joint_pos)
    )
    scene.reset_to(prev_state, env_ids=None)
    assert_state_equal(prev_state, scene.get_state())

    # test env_ids = torch tensor
    scene["robot"].write_joint_state_to_sim(
        position=torch.rand_like(scene["robot"].data.joint_pos), velocity=torch.rand_like(scene["robot"].data.joint_pos)
    )
    scene.reset_to(prev_state, env_ids=torch.arange(scene.num_envs, device=scene.device))
    assert_state_equal(prev_state, scene.get_state())


def assert_state_equal(s1: dict, s2: dict, path=""):
    """
    Recursively assert that s1 and s2 have the same nested keys
    and that every tensor leaf is exactly equal.
    """
    assert set(s1.keys()) == set(s2.keys()), f"Key mismatch at {path}: {s1.keys()} vs {s2.keys()}"
    for k in s1:
        v1, v2 = s1[k], s2[k]
        subpath = f"{path}.{k}" if path else k
        if isinstance(v1, dict):
            assert isinstance(v2, dict), f"Type mismatch at {subpath}"
            assert_state_equal(v1, v2, path=subpath)
        else:
            # leaf: should be a torch.Tensor
            assert isinstance(v1, torch.Tensor) and isinstance(v2, torch.Tensor), f"Expected tensors at {subpath}"
            if not torch.equal(v1, v2):
                diff = (v1 - v2).abs().max()
                pytest.fail(f"Tensor mismatch at {subpath}, max abs diff = {diff}")


def assert_state_different(s1: dict, s2: dict, path=""):
    """
    Recursively scan s1 and s2 (which must have identical keys) and
    succeed as soon as you find one tensor leaf that differs.
    If you reach the end with everything equal, fail the test.
    """
    assert set(s1.keys()) == set(s2.keys()), f"Key mismatch at {path}: {s1.keys()} vs {s2.keys()}"
    for k in s1:
        v1, v2 = s1[k], s2[k]
        subpath = f"{path}.{k}" if path else k
        if isinstance(v1, dict):
            # recurse; if any nested call returns (i.e. finds a diff), we propagate success
            try:
                assert_state_different(v1, v2, path=subpath)
                return
            except AssertionError:
                continue
        else:
            assert isinstance(v1, torch.Tensor) and isinstance(v2, torch.Tensor), f"Expected tensors at {subpath}"
            if not torch.equal(v1, v2):
                return  # found a difference → success
    pytest.fail(f"No differing tensor found in nested state at {path}")
=======
>>>>>>> abfba5273e (Fresh start, no history)
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
