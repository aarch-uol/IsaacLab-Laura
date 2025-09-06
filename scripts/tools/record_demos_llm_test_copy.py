# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to record demonstrations with Isaac Lab environments using human teleoperation.

This script allows users to record demonstrations operated by human teleoperation for a specified task.
The recorded demonstrations are stored as episodes in a hdf5 file. Users can specify the task, teleoperation
device, dataset directory, and environment stepping rate through command-line arguments.

required arguments:
    --task                    Name of the task.

optional arguments:
    -h, --help                Show this help message and exit
    --teleop_device           Device for interacting with environment. (default: keyboard)
    --dataset_file            File path to export recorded demos. (default: "./datasets/dataset.hdf5")
    --step_hz                 Environment stepping rate in Hz. (default: 30)
    --num_demos               Number of demonstrations to record. (default: 0)
    --num_success_steps       Number of continuous steps with task success for concluding a demo as successful. (default: 10)
"""

"""Launch Isaac Sim Simulator first."""

# Standard library imports
import argparse
import contextlib

# Third-party imports
import gymnasium as gym
import numpy as np
import os
import time
import torch

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Record demonstrations for Isaac Lab environments.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--teleop_device", type=str, default="keyboard", help="Device for interacting with environment.")
parser.add_argument(
    "--dataset_file", type=str, default="./datasets/dataset.hdf5", help="File path to export recorded demos."
)
parser.add_argument("--step_hz", type=int, default=30, help="Environment stepping rate in Hz.")
parser.add_argument(
    "--num_demos", type=int, default=0, help="Number of demonstrations to record. Set to 0 for infinite."
)
parser.add_argument(
    "--num_success_steps",
    type=int,
    default=10,
    help="Number of continuous steps with task success for concluding a demo as successful. Default is 10.",
)
parser.add_argument(
    "--enable_pinocchio",
    action="store_true",
    default=False,
    help="Enable Pinocchio.",
)
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

app_launcher_args = vars(args_cli)

if args_cli.enable_pinocchio:
    # Import pinocchio before AppLauncher to force the use of the version installed by IsaacLab and not the one installed by Isaac Sim
    # pinocchio is required by the Pink IK controllers and the GR1T2 retargeter
    import pinocchio  # noqa: F401

# launch the simulator
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Omniverse logger
import omni.log
import omni.ui as ui


import isaaclab_mimic.envs  # noqa: F401
from isaaclab_mimic.ui.instruction_display import InstructionDisplay, show_subtask_instructions

if args_cli.enable_pinocchio:
    from isaaclab.devices.openxr.retargeters.humanoid.fourier.gr1t2_retargeter import GR1T2Retargeter
    import isaaclab_tasks.manager_based.manipulation.pick_place  # noqa: F401

from isaaclab.devices.openxr.retargeters.manipulator import GripperRetargeter, Se3AbsRetargeter, Se3RelRetargeter
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg
from isaaclab.envs.ui import EmptyWindow
from isaaclab.managers import DatasetExportMode

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg
#from scripts.environments.state_machine.stack_lab_sm import PickAndLiftSm
# from scripts.environments.state_machine.weigh_lab_sm import PickAndLiftSm
#from scripts.environments.state_machine.pour_lab_sm import PickAndLiftSm
from scripts.environments.state_machine.stack_weigh_lab_sm import PickAndLiftSm
import random
import numpy as np

class RateLimiter:
    """Convenience class for enforcing rates in loops."""

    def __init__(self, hz: int):
        """Initialize a RateLimiter with specified frequency.

        Args:
            hz: Frequency to enforce in Hertz.
        """
        self.hz = hz
        self.last_time = time.time()
        self.sleep_duration = 1.0 / hz
        self.render_period = min(0.033, self.sleep_duration)

    def sleep(self, env: gym.Env):
        """Attempt to sleep at the specified rate in hz.

        Args:
            env: Environment to render during sleep periods.
        """
        next_wakeup_time = self.last_time + self.sleep_duration
        while time.time() < next_wakeup_time:
            time.sleep(self.render_period)
            env.sim.render()

        self.last_time = self.last_time + self.sleep_duration

        # detect time jumping forwards (e.g. loop is too slow)
        if self.last_time < time.time():
            while self.last_time < time.time():
                self.last_time += self.sleep_duration

def _ensure_idx_list(idx, device):
    """Return a python list and a torch.LongTensor on device for the given index(s)."""
    if isinstance(idx, torch.Tensor):
        idx_list = idx.detach().cpu().tolist()
    elif isinstance(idx, (list, tuple)):
        idx_list = list(idx)
    else:
        idx_list = [int(idx)]
    idx_tensor = torch.tensor(idx_list, dtype=torch.long, device=device)
    return idx_list, idx_tensor


def ensure_actions_valid(actions, env):
    """Return actions as a torch tensor on the env device with correct shape/dtype."""
    # convert numpy or list to tensor
    if not isinstance(actions, torch.Tensor):
        actions = torch.tensor(actions)
    # cast and move to device
    actions = actions.to(device=env.unwrapped.device, dtype=torch.float32)
    # ensure shape matches action_space
    expected_shape = env.unwrapped.action_space.shape
    if actions.shape != expected_shape:
        # try to reshape/broadcast in a safe way
        actions = actions.view(expected_shape)
    return actions


def init_state_machine(env, env_cfg):
    """Create PickAndLiftSm once and capture initial TCP orientation."""

    pick_sm = PickAndLiftSm(
        env_cfg.sim.dt * env_cfg.decimation,
        env.unwrapped.num_envs,
        env.unwrapped.device,
        position_threshold=0.01,
    )

    # Read current ee frame and capture a fixed orientation (per env)
    ee_frame = env.unwrapped.scene["ee_frame"]
    fixed_tcp_rot = ee_frame.data.target_quat_w[..., 0, :].clone() 
    print(f"init state machine : fixed tcp rot : {fixed_tcp_rot}")
    return pick_sm, fixed_tcp_rot

def state_machine_step(env, pick_sm, fixed_tcp_rot):
    """Compute a single-step action from the persistent state machine (does NOT call env.step)."""
    with torch.inference_mode():
        ee_frame = env.unwrapped.scene["ee_frame"]
        tcp_pos = ee_frame.data.target_pos_w[..., 0, :].clone() - env.unwrapped.scene.env_origins
       # print(f"state machine rot  : {ee_frame.data.target_pos_w[..., 0, :].clone()}")
        obj1_pos = env.unwrapped.scene["object1"].data.root_pos_w - env.unwrapped.scene.env_origins
        obj2_pos = env.unwrapped.scene["object2"].data.root_pos_w - env.unwrapped.scene.env_origins
        obj3_pos = env.unwrapped.scene["object3"].data.root_pos_w - env.unwrapped.scene.env_origins
        desired_pos = env.unwrapped.command_manager.get_command("object_pose")[..., :3]
       # print(f"State machine step using fixed rotation : {fixed_tcp_rot}")
        # Use the fixed quaternion for all inputs so orientation is locked.
        a = pick_sm.compute(
            torch.cat([tcp_pos, fixed_tcp_rot], dim=-1),
            torch.cat([obj1_pos, fixed_tcp_rot], dim=-1),
            torch.cat([desired_pos, fixed_tcp_rot], dim=-1),
            torch.cat([obj2_pos,  fixed_tcp_rot], dim=-1),
            torch.cat([obj3_pos, fixed_tcp_rot], dim=-1),
        )

    return a

def reset_env_and_sm(env, pick_sm, reset_indices=None, fixed_tcp_rot=None, actions= None):
    """Reset specified environments (or all if None) along with the state machine."""
    #seed = random.randint(1, 100)
    if reset_indices is None:
        # full reset
        print("[RESET] Performing full environment reset")
        env.reset()
        reset_indices = torch.arange(env.unwrapped.num_envs, device=env.unwrapped.device)
    else:
        # partial reset
        if isinstance(reset_indices, int):
            reset_indices = torch.tensor([reset_indices], device=env.unwrapped.device)
        elif isinstance(reset_indices, list):
            reset_indices = torch.tensor(reset_indices, device=env.unwrapped.device)

        print("Resetting env id : ", reset_indices.tolist())
        try:
            #print(f"[RESET] Attempting env.reset(reset_indices={reset_indices.tolist()})")
            env.reset(env_ids=reset_indices)
        except TypeError:
           # print(f"[RESET] Fallback: env.reset(indices={reset_indices.tolist()})")
            env.reset(env_ids=reset_indices)

    # reset state machine for these envs
    pick_sm.reset_idx(reset_indices)

    # get new tcp rotation for *all* envs (keep shape consistent)
    ee_frame = env.unwrapped.scene["ee_frame"]
    
    if fixed_tcp_rot is None:
        # initialize for the first time
        # [num_envs , 4] tensor for rotation
        reset_rot = ee_frame.data.target_quat_w[..., 0, :].clone()
        print("first fixed_tcp_rot : ", reset_rot)
    else:
        reset_rot = fixed_tcp_rot
        # fixed_pos_idx = fixed_tcp_rot[reset_indices]
        # print(f"Get fixed pos for id {reset_indices} : {fixed_pos_idx} ")
        # reset_rot = ee_frame.data.target_quat_w[..., 0, :].clone()
        # reset_rot[reset_indices] = fixed_pos_idx
        # print(f"fixed_tcp_pos : {reset_rot}, shape {reset_rot.shape}")

        
    #     new_tcp_rot = ee_frame.data.target_quat_w[..., reset_indices, :].squeeze(-2).clone()
    #     print("new_tcp_rot  : ", new_tcp_rot.shape())
    #     fixed_tcp_rot = new_tcp_rot
    # reset action buffer (neutral gripper open)
    if actions is None:
        actions = torch.zeros(env.unwrapped.action_space.shape, device=env.unwrapped.device)
        actions[:, 3] = 1.0
    else :
       # print(f"current actions : {actions}, i need to replace {actions[reset_indices]} with a tensor {actions[reset_indices.shape]}")
        new_action_holder = actions.clone()
        new_action_holder[reset_indices] = torch.zeros(8, device=env.unwrapped.device)
        new_action_holder[reset_indices, 3] = 1.0
        actions = new_action_holder
        #print(f"after reset action : {actions}")
   # print(f"POST RESET tcp {fixed_tcp_rot}, actions {actions}, ")
    print(f"[RESET] Completed reset for indices {reset_indices.tolist()}")
    return reset_rot, actions


def main():
    """Collect demonstrations from the environment using teleop interfaces."""

    # if handtracking is selected, rate limiting is achieved via OpenXR
    if args_cli.headless:
        rate_limiter = None
    else:
        rate_limiter = RateLimiter(args_cli.step_hz)

    # get directory path and file name (without extension) from cli arguments
    output_dir = os.path.dirname(args_cli.dataset_file)
    output_file_name = os.path.splitext(os.path.basename(args_cli.dataset_file))[0]
    os.makedirs(output_dir, exist_ok=True)

    num_envs = 400
    global_success_count = 0

    # parse configuration
    env_cfg: LiftEnvCfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=num_envs,
        use_fabric=not args_cli.disable_fabric,
    )

    # configure dataset recording
    env_cfg.recorders: ActionStateRecorderManagerCfg = ActionStateRecorderManagerCfg()
    env_cfg.recorders.dataset_export_dir_path = output_dir
    env_cfg.recorders.dataset_filename = output_file_name
    env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY

    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # reset before starting
    env.sim.reset()
    env.reset()

    # UI / labels
    label_text = f"Recorded {global_success_count} successful demonstrations."
    if not args_cli.headless:
        instruction_display = InstructionDisplay(args_cli.teleop_device)
        window = EmptyWindow(env, "Instruction")
        with window.ui_window_elements["main_vstack"]:
            demo_label = ui.Label(label_text)
            subtask_label = ui.Label("")
            instruction_display.set_labels(subtask_label, demo_label)
    else:
        demo_label = None
        subtask_label = None
        print(label_text)

    # init state machine
    pick_sm, fixed_tcp_rot = init_state_machine(env, env_cfg)
    #print(f"init fixed tcp {fixed_tcp_rot}")
    fixed_tcp_rot, actions = reset_env_and_sm(env, pick_sm)
   # print(f"BEFORE fixed tcp {fixed_tcp_rot} and actions {actions}")
    print(f"Recorded {env.recorder_manager.exported_successful_episode_count} successful demonstrations.")
    while simulation_app.is_running():
        obs, rewards, terminated, truncated, infos = env.step(actions)
       # print("terms : ", terminated)
        success_mask = obs["policy"]["success_obs"]
        #print("success obs : ", success_mask)
        #print("infos: ", infos)
        dones = (terminated | truncated) if isinstance(terminated, torch.Tensor) else np.logical_or(terminated, truncated)
        actions = state_machine_step(env, pick_sm, fixed_tcp_rot)

        # convert done mask
        done_mask = (
            dones.detach().cpu().numpy().astype(bool)
            if isinstance(dones, torch.Tensor)
            else np.asarray(dones, dtype=bool)
        )
        #print("Done mask ", done_mask)
        reset_indices = []
        
        for idx in range(env.unwrapped.num_envs):
            if done_mask[idx]:
                print(f"Env {idx} finished ")
                reset_idx_tensor = torch.tensor([idx], device=env.device)
                # just store all - success seems to lag
                global_success_count += 1
                print(f"[INFO] ✅ Success in env {idx} (total successes: {global_success_count})")

                env.recorder_manager.record_pre_reset(reset_idx_tensor, force_export_or_skip=False)
                env.recorder_manager.set_success_to_episodes(
                    reset_idx_tensor,
                    torch.tensor([[True]], dtype=torch.bool, device=env.device),
                )
                
                env.recorder_manager.export_episodes(reset_idx_tensor)
                reset_indices.append(idx)
                reset_idx_tensor = torch.tensor([idx], device=env.device)
                fixed_tcp_rot, actions = reset_env_and_sm(env, pick_sm, reset_idx_tensor, fixed_tcp_rot=fixed_tcp_rot, actions=actions)    
              #  print(f"AFTER fixed tcp {fixed_tcp_rot} and actions {actions}")
        if not args_cli.headless:
            env.sim.render()

        # stop when enough demos recorded
        if args_cli.num_demos > 0 and global_success_count >= args_cli.num_demos:
            print(f"All {args_cli.num_demos} demonstrations recorded. Exiting the app.")
            break

        if env.sim.is_stopped():
            break

        if rate_limiter:
            rate_limiter.sleep(env)

    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
