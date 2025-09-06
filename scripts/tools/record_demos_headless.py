# Copyright (c) 2024-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Headless script to record demonstrations with Isaac Lab environments using human teleoperation.

This version is optimized for speed:
- Runs headless (no UI, no rendering)
- Keeps per-environment success streak tracking
- Integrates environment terminations, but handles "success_term" manually
"""

# Standard library
import argparse
import os
import time
import numpy as np
import torch
import gymnasium as gym

# Isaac Lab AppLauncher
from isaaclab.app import AppLauncher

# CLI args
parser = argparse.ArgumentParser(description="Record demonstrations for Isaac Lab environments (headless).")
parser.add_argument("--task", type=str, required=True, help="Name of the task.")
parser.add_argument("--dataset_file", type=str, default="./datasets/dataset.hdf5", help="Output dataset file.")
parser.add_argument("--step_hz", type=int, default=30, help="Simulation step rate in Hz.")
parser.add_argument("--num_demos", type=int, default=0, help="Number of demos to record (0 = infinite).")
parser.add_argument("--num_success_steps", type=int, default=10,
                    help="Continuous success steps needed to count a demo as successful.")
parser.add_argument("--enable_pinocchio", action="store_true", default=False, help="Enable Pinocchio.")
parser.add_argument("--disable_fabric", action="store_true", default=False, help="Disable fabric.")
parser.add_argument("--num_envs", type=int, default=40, help="Number of parallel environments.")
#parser.add_argument("--headless", action="store_true", default=True, help="Force headless mode.")

# AppLauncher args
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Omniverse logger
import omni.log

# Isaac Lab imports
import isaaclab_mimic.envs  # noqa: F401
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg
from isaaclab.managers import DatasetExportMode
import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg
from scripts.environments.state_machine.stack_weigh_lab_sm import PickAndLiftSm


class RateLimiter:
    """Enforce a fixed loop rate."""

    def __init__(self, hz: int):
        self.sleep_duration = 1.0 / hz
        self.last_time = time.time()

    def sleep(self):
        next_wakeup = self.last_time + self.sleep_duration
        while time.time() < next_wakeup:
            time.sleep(0.0005)
        self.last_time += self.sleep_duration
        if self.last_time < time.time():
            while self.last_time < time.time():
                self.last_time += self.sleep_duration


def init_state_machine(env, env_cfg):
    """Create PickAndLiftSm and capture initial TCP orientation."""
    pick_sm = PickAndLiftSm(
        env_cfg.sim.dt * env_cfg.decimation,
        env.unwrapped.num_envs,
        env.unwrapped.device,
        position_threshold=0.01,
    )
    ee_frame = env.unwrapped.scene["ee_frame"]
    fixed_tcp_rot = ee_frame.data.target_quat_w[..., 0, :].clone()
    return pick_sm, fixed_tcp_rot


def state_machine_step(env, pick_sm, fixed_tcp_rot):
    """Compute one step of action from the state machine."""
    with torch.inference_mode():
        ee_frame = env.unwrapped.scene["ee_frame"]
        tcp_pos = ee_frame.data.target_pos_w[..., 0, :] - env.unwrapped.scene.env_origins
        obj1_pos = env.unwrapped.scene["object1"].data.root_pos_w - env.unwrapped.scene.env_origins
        obj2_pos = env.unwrapped.scene["object2"].data.root_pos_w - env.unwrapped.scene.env_origins
        obj3_pos = env.unwrapped.scene["object3"].data.root_pos_w - env.unwrapped.scene.env_origins
        desired_pos = env.unwrapped.command_manager.get_command("object_pose")[..., :3]

        a = pick_sm.compute(
            torch.cat([tcp_pos, fixed_tcp_rot], dim=-1),
            torch.cat([obj1_pos, fixed_tcp_rot], dim=-1),
            torch.cat([desired_pos, fixed_tcp_rot], dim=-1),
            torch.cat([obj2_pos, fixed_tcp_rot], dim=-1),
            torch.cat([obj3_pos, fixed_tcp_rot], dim=-1),
        )
    return a


def reset_env_and_sm(env, pick_sm, reset_indices=None):
    """Reset envs (all or subset) and the state machine."""
    if reset_indices is None:
        env.sim.reset()
        env.reset()
        reset_indices = list(range(env.unwrapped.num_envs))
    else:
        # ensure Python list for env.reset
        if isinstance(reset_indices, torch.Tensor):
            reset_indices = reset_indices.tolist()
        elif not isinstance(reset_indices, (list, tuple, np.ndarray)):
            reset_indices = [int(reset_indices)]
        env.reset(reset_indices)

    # pick_sm still wants torch tensor indices
    reset_indices_tensor = torch.as_tensor(reset_indices, device=env.unwrapped.device, dtype=torch.long)
    pick_sm.reset_idx(reset_indices_tensor)

    ee_frame = env.unwrapped.scene["ee_frame"]
    fixed_tcp_rot = ee_frame.data.target_quat_w[..., 0, :].clone()

    actions = torch.zeros(env.unwrapped.action_space.shape, device=env.unwrapped.device)
    actions[:, 3] = 1.0
    return fixed_tcp_rot, actions





def main():
    # dataset dir
    output_dir = os.path.dirname(args_cli.dataset_file)
    output_file_name = os.path.splitext(os.path.basename(args_cli.dataset_file))[0]
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # parse cfg
    env_cfg: LiftEnvCfg = parse_env_cfg(
        args_cli.task,
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )

    # pull success termination manually
    success_term = None
    if hasattr(env_cfg.terminations, "success_term"):
        success_term = env_cfg.terminations.success_term
        env_cfg.terminations.success_term = None
    else:
        omni.log.warn("No success termination found; demos will never be marked successful.")

    # recorder config
    env_cfg.recorders: ActionStateRecorderManagerCfg = ActionStateRecorderManagerCfg()
    env_cfg.recorders.dataset_export_dir_path = output_dir
    env_cfg.recorders.dataset_filename = output_file_name
    env_cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY

    # make env
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
    env.sim.reset()
    env.reset()

    # setup helpers
    rate_limiter = RateLimiter(args_cli.step_hz)
    pick_sm, fixed_tcp_rot = init_state_machine(env, env_cfg)
    fixed_tcp_rot, actions = reset_env_and_sm(env, pick_sm)

    success_step_counts = [0 for _ in range(env.unwrapped.num_envs)]
    current_recorded_demo_count = 0

    print(f"[INFO] Recording demos into {args_cli.dataset_file}")

    # --- inside main(), replace your big while loop with this version ---
    while simulation_app.is_running():
    # step env
        obs, rewards, dones, truncations, infos = env.step(actions)
        actions = state_machine_step(env, pick_sm, fixed_tcp_rot)

        if dones.any():
            # Convert dones to boolean mask (per-env)
            done_mask = dones.detach().cpu().numpy().astype(bool) if isinstance(dones, torch.Tensor) else np.asarray(dones, dtype=bool)

            # Evaluate success termination per env
            if success_term is not None:
                success_mask = success_term.func(env, **success_term.params)  # per-env tensor
                success_mask = success_mask.detach().cpu().numpy().astype(bool)
            else:
                success_mask = np.zeros(env.unwrapped.num_envs, dtype=bool)

            exported_any = False
            exported_indices = []

            for idx in range(env.unwrapped.num_envs):
                if not done_mask[idx]:
                    continue  # skip envs still running

                if success_mask[idx]:
                    success_step_counts[idx] += 1
                    print(f"[INFO] Env {idx} success streak: {success_step_counts[idx]}")
                else:
                    if success_step_counts[idx] != 0:
                        print(f"[INFO] Env {idx} failed, resetting streak")
                    success_step_counts[idx] = 0

                # reached threshold → export
                if success_step_counts[idx] >= args_cli.num_success_steps:
                    print(f"[INFO] Exporting successful demo for env {idx}")
                    env.recorder_manager.record_pre_reset([idx], force_export_or_skip=False)
                    env.recorder_manager.set_success_to_episodes(
                        [idx],
                        torch.tensor([[True]], dtype=torch.bool, device=env.device)
                    )
                    env.recorder_manager.export_episodes([idx])
                    exported_any = True
                    exported_indices.append(idx)
                    success_step_counts[idx] = 0  # reset after export

            if exported_any:
                print(f"[INFO] Exported env indices: {exported_indices}")
                fixed_tcp_rot, actions = reset_env_and_sm(
                    env, pick_sm, list(exported_indices)  # ✅ force Python list
                )
                for idx in exported_indices:
                    success_step_counts[idx] = 0
            else:
                reset_no_export_ids = [idx for idx in range(env.unwrapped.num_envs) if done_mask[idx]]
                if reset_no_export_ids:
                    print(f"[INFO] Resetting envs without export: {reset_no_export_ids}")
                    fixed_tcp_rot, actions = reset_env_and_sm(
                        env, pick_sm, list(reset_no_export_ids)  # ✅ force Python list
                    )
                    for idx in reset_no_export_ids:
                        success_step_counts[idx] = 0

        # check demo count
        if env.recorder_manager.exported_successful_episode_count > current_recorded_demo_count:
            current_recorded_demo_count = env.recorder_manager.exported_successful_episode_count
            print(f"[INFO] Recorded {current_recorded_demo_count} successful demos.")

        if args_cli.num_demos > 0 and current_recorded_demo_count >= args_cli.num_demos:
            print(f"[INFO] Collected {args_cli.num_demos} demos. Exiting.")
            break

        if env.sim.is_stopped():
            break

        rate_limiter.sleep()



    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
