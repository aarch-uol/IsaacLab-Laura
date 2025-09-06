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

    return pick_sm, fixed_tcp_rot

def state_machine_step(env, pick_sm, fixed_tcp_rot):
    """Compute a single-step action from the persistent state machine (does NOT call env.step)."""
    with torch.inference_mode():
        ee_frame = env.unwrapped.scene["ee_frame"]
        tcp_pos = ee_frame.data.target_pos_w[..., 0, :].clone() - env.unwrapped.scene.env_origins
        print(f"state machine rot  : {ee_frame.data.target_pos_w[..., 0, :].clone()}")
        obj1_pos = env.unwrapped.scene["object1"].data.root_pos_w - env.unwrapped.scene.env_origins
        obj2_pos = env.unwrapped.scene["object2"].data.root_pos_w - env.unwrapped.scene.env_origins
        # obj3_pos = env.unwrapped.scene["object3"].data.root_pos_w - env.unwrapped.scene.env_origins
        desired_pos = env.unwrapped.command_manager.get_command("object_pose")[..., :3]

        # Use the fixed quaternion for all inputs so orientation is locked.
        a = pick_sm.compute(
            torch.cat([tcp_pos, fixed_tcp_rot], dim=-1),
            torch.cat([obj1_pos, fixed_tcp_rot], dim=-1),
            torch.cat([desired_pos, fixed_tcp_rot], dim=-1),
            torch.cat([obj2_pos,  fixed_tcp_rot], dim=-1),
            # torch.cat([obj3_pos, fixed_tcp_rot], dim=-1),
        )

    return a


def main():
    """Collect demonstrations from the environment using teleop interfaces."""

    # if handtracking is selected, rate limiting is achieved via OpenXR
    rate_limiter = RateLimiter(args_cli.step_hz)

    # get directory path and file name (without extension) from cli arguments
    output_dir = os.path.dirname(args_cli.dataset_file)
    output_file_name = os.path.splitext(os.path.basename(args_cli.dataset_file))[0]

    # create directory if it does not exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # parse configuration
    env_cfg: LiftEnvCfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=1,
        use_fabric=not args_cli.disable_fabric,
    )

    # extract success checking function to invoke in the main loop
    success_term = None
    if hasattr(env_cfg.terminations, "success_term"):
        success_term = env_cfg.terminations.success_term
        env_cfg.terminations.success_term = None
    else:
        omni.log.warn(
            "No success termination term was found in the environment."
            " Will not be able to mark recorded demos as successful."
        )

    # modify configuration such that the environment runs indefinitely until
    # the goal is reached or other termination conditions are met
    env_cfg.recorders: ActionStateRecorderManagerCfg = ActionStateRecorderManagerCfg()
    env_cfg.recorders.dataset_export_dir_path = output_dir
    env_cfg.recorders.dataset_filename = output_file_name
    env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY

    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    env.reset()

    # Flags for controlling the demonstration recording process
    should_reset_recording_instance = False
    running_recording_instance = True

    def reset_recording_instance():
        """Reset the current recording instance.

        This function is triggered when the user indicates the current demo attempt
        has failed and should be discarded. When called, it marks the environment
        for reset, which will start a fresh recording instance. This is useful when:
        - The robot gets into an unrecoverable state
        - The user makes a mistake during demonstration
        - The objects in the scene need to be reset to their initial positions
        """
        nonlocal should_reset_recording_instance
        should_reset_recording_instance = True

    def start_recording_instance():
        """Start or resume recording the current demonstration.

        This function enables active recording of robot actions. It's used when:
        - Beginning a new demonstration after positioning the robot
        - Resuming recording after temporarily stopping to reposition
        - Continuing demonstration after pausing to adjust approach or strategy

        The user can toggle between stop/start to reposition the robot without
        recording those transitional movements in the final demonstration.
        """
        nonlocal running_recording_instance
        running_recording_instance = True

    def stop_recording_instance():
        """Temporarily stop recording the current demonstration.

        This function pauses the active recording of robot actions, allowing the user to:
        - Reposition the robot or hand tracking device without recording those movements
        - Take a break without terminating the entire demonstration
        - Adjust their approach before continuing with the task

        The environment will continue rendering but won't record actions or advance
        the simulation until recording is resumed with start_recording_instance().
        """
        nonlocal running_recording_instance
        running_recording_instance = False

    # reset before starting
    env.sim.reset()
    env.reset()

    # simulate environment -- run everything in inference mode
    current_recorded_demo_count = 0
    success_step_count = 0

    label_text = f"Recorded {current_recorded_demo_count} successful demonstrations."

    instruction_display = InstructionDisplay(args_cli.teleop_device)
    window = EmptyWindow(env, "Instruction")
    with window.ui_window_elements["main_vstack"]:
        demo_label = ui.Label(label_text)
        subtask_label = ui.Label("")
        instruction_display.set_labels(subtask_label, demo_label)

    pick_sm, fixed_tcp_rot = init_state_machine(env, env_cfg)
    actions = torch.zeros(env.unwrapped.action_space.shape, device=env.unwrapped.device)
    actions[:, 3] = 1.0

    # with contextlib.suppress(KeyboardInterrupt) and torch.inference_mode():
    while simulation_app.is_running():
        if running_recording_instance:
            print(f"[RECORDED] action stepped into env: {actions}")
            dones = env.step(actions)[-2]
            actions = state_machine_step(env, pick_sm, fixed_tcp_rot)
            print(f"[DEBUG] state machine actions : {actions}")
            if dones.any():
                # ✅ check success condition
                is_success = False
                if success_term is not None:
                    is_success = bool(success_term.func(env, **success_term.params)[0])

                if is_success:
                    success_step_count += 1
                    if success_step_count >= args_cli.num_success_steps:
                        env.recorder_manager.record_pre_reset([0], force_export_or_skip=False)
                        env.recorder_manager.set_success_to_episodes(
                            [0], torch.tensor([[True]], dtype=torch.bool, device=env.device)
                        )
                        env.recorder_manager.export_episodes([0])
                        print("✅ Successful demo exported")

                        # full reset
                        env.sim.reset()
                        env.reset()
                        pick_sm.reset_idx(torch.arange(env.unwrapped.num_envs, device=env.unwrapped.device))
                        ee_frame = env.unwrapped.scene["ee_frame"]
                        fixed_tcp_rot = ee_frame.data.target_quat_w[..., 0, :].clone()
                        success_step_count = 0
                else:
                    # ❌ fail → reset but don’t export
                    print("⚠️ Failed demo, resetting without export")
                    env.sim.reset()
                    env.reset()
                    pick_sm.reset_idx(torch.arange(env.unwrapped.num_envs, device=env.unwrapped.device))
                    ee_frame = env.unwrapped.scene["ee_frame"]
                    fixed_tcp_rot = ee_frame.data.target_quat_w[..., 0, :].clone()
                    success_step_count = 0

            else:
                env.sim.render()

        if env.recorder_manager.exported_successful_episode_count > current_recorded_demo_count:
            current_recorded_demo_count = env.recorder_manager.exported_successful_episode_count
            label_text = f"Recorded {current_recorded_demo_count} successful demonstrations."
            print(label_text)

        if args_cli.num_demos > 0 and env.recorder_manager.exported_successful_episode_count >= args_cli.num_demos:
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
