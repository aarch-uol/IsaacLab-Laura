# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

# run this script: 
# ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py --device cuda --task Dev-IK-Rel-v0 --num_rollouts 50
# stack cube task: ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py --device cuda --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --num_rollouts 50
"""Script to play and evaluate a trained policy from robomimic.

This script loads a robomimic policy and plays it in an Isaac Lab environment.

Args:
    task: Name of the environment.
    checkpoint: Path to the robomimic policy checkpoint.
    horizon: If provided, override the step horizon of each rollout.
    num_rollouts: If provided, override the number of rollouts.
    seed: If provided, overeride the default random seed.
    norm_factor_min: If provided, minimum value of the action space normalization factor.
    norm_factor_max: If provided, maximum value of the action space normalization factor.
"""

"""Launch Isaac Sim Simulator first."""


import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Evaluate robomimic policy for Isaac Lab environment.")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument("--checkpoint", type=str, default=None, help="Pytorch model checkpoint to load.")
parser.add_argument("--horizon", type=int, default=800, help="Step horizon of each rollout.")
parser.add_argument("--num_rollouts", type=int, default=1, help="Number of rollouts.")
parser.add_argument("--seed", type=int, default=101, help="Random seed.")
parser.add_argument(
    "--norm_factor_min", type=float, default=None, help="Optional: minimum value of the normalization factor."
)
parser.add_argument(
    "--norm_factor_max", type=float, default=None, help="Optional: maximum value of the normalization factor."
)
parser.add_argument("--enable_pinocchio", default=False, action="store_true", help="Enable Pinocchio.")

parser.add_argument("--model_name", type=str, default=None)

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

if args_cli.enable_pinocchio:
    # Import pinocchio before AppLauncher to force the use of the version installed by IsaacLab and not the one installed by Isaac Sim
    # pinocchio is required by the Pink IK controllers and the GR1T2 retargeter
    import pinocchio  # noqa: F401

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import time
import copy
import random
import os
import gymnasium as gym
import torch
from collections import deque

import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils

if args_cli.enable_pinocchio:
    import isaaclab_tasks.manager_based.manipulation.pick_place  # noqa: F401

from isaaclab_tasks.utils import parse_env_cfg
from isaaclab.utils.logging_helper import LoggingHelper, ErrorType, LogType


from evaluation import inject_dropout_layers, MC_dropout_uncertainty, remove_dropout_layers, ensemble_uncertainty



def rollout(policy, env, success_term, horizon, device):
    """Perform a single rollout of the policy in the environment.

    Args:
        policy: The robomimicpolicy to play.
        env: The environment to play in.
        horizon: The step horizon of each rollout.
        device: The device to run the policy on.

    Returns:
        terminated: Whether the rollout terminated.
        traj: The trajectory of the rollout.
    """
    policy.start_episode()
    obs_dict, _ = env.reset()
    
    #print(f"obs_dict type: {type(obs_dict)},\nobs_dict contents:\n {obs_dict}")

    traj = dict(actions=[], obs=[], next_obs=[], sub_obs=[], uncertainties=[])

    for i in range(horizon):
        # Prepare observations
        obs = copy.deepcopy(obs_dict["policy"])
        sub_obs = copy.deepcopy(obs_dict["subtask_terms"])
       
        for ob in obs:
            obs[ob] = torch.squeeze(obs[ob])
            #print(f"found observation : {obs[ob]}")
        
        for subob in sub_obs:
            sub_obs[subob] = torch.squeeze(sub_obs[subob])

        # Check if environment image observations
        if hasattr(env.cfg, "image_obs_list"):
            # Process image observations for robomimic inference
            for image_name in env.cfg.image_obs_list:
                if image_name in obs_dict["policy"].keys():
                    # Convert from chw uint8 to hwc normalized float
                    image = torch.squeeze(obs_dict["policy"][image_name])
                    image = image.permute(2, 0, 1).clone().float()
                    image = image / 255.0
                    image = image.clip(0.0, 1.0)
                    obs[image_name] = image

        traj["obs"].append(obs)
        traj["sub_obs"].append(sub_obs)
        
        # Add dropout layers to the model and calculate uncertainty and remove at the end to not effect final action

        #hooks = inject_dropout_layers(policy=policy, probability=0.1)
        # uncertainty_dict = MC_dropout_uncertainty(policy=policy, obs=obs, niters=15)
        # traj['uncertainties'].append(uncertainty_dict['variance'])
        #remove_dropout_layers(hooks)

        # Compute actions
        actions = policy(obs)
      
        # Unnormalize actions
        if args_cli.norm_factor_min is not None and args_cli.norm_factor_max is not None:
            actions = (
                (actions + 1) * (args_cli.norm_factor_max - args_cli.norm_factor_min)
            ) / 2 + args_cli.norm_factor_min

        actions = torch.from_numpy(actions).to(device=device).view(1, env.action_space.shape[1])

        # Apply actions
        obs_dict, _, terminated, truncated, _ = env.step(actions)
        obs = obs_dict["policy"]
        sub_obs = obs_dict["subtask_terms"]

        # Record trajectory
        traj["actions"].append(actions.tolist())
        traj["next_obs"].append(obs)


        # Check if rollout was successful
        if bool(success_term.func(env, **success_term.params)[0]):
            return True, traj
        elif terminated or truncated:
            return False, traj

    return False, traj


def rollout_ensemble(ensemble, env, success_term, horizon, device, parameters):
    """Perform a single rollout of the policy in the environment.

    Args:
        policy: The robomimicpolicy to play.
        env: The environment to play in.
        horizon: The step horizon of each rollout.
        device: The device to run the policy on.

    Returns:
        terminated: Whether the rollout terminated.
        traj: The trajectory of the rollout.
    """

    for policy in ensemble:
        policy.start_episode()
    obs_dict, _ = env.reset()
    
    #print(f"obs_dict type: {type(obs_dict)},\nobs_dict contents:\n {obs_dict}")

    traj = dict(actions=[], obs=[], next_obs=[], sub_obs=[], 
                uncertainties=[], min_actions=[], max_actions=[])
    
    joints_to_check = [0, 1, 2, 3, 4, 5, 6]
    unsafe_windows_detected = {joint_num: 0 for joint_num in joints_to_check}
    windows = {joint_num: deque(maxlen=parameters[joint_num]['window_size']) for joint_num in joints_to_check} 
    
    for i in range(horizon):
        # Prepare observations
        obs = copy.deepcopy(obs_dict["policy"])
        sub_obs = copy.deepcopy(obs_dict["subtask_terms"])
       
        for ob in obs:
            obs[ob] = torch.squeeze(obs[ob])
            #print(f"found observation : {obs[ob]}")
        
        for subob in sub_obs:
            sub_obs[subob] = torch.squeeze(sub_obs[subob])

        # Check if environment image observations
        if hasattr(env.cfg, "image_obs_list"):
            # Process image observations for robomimic inference
            for image_name in env.cfg.image_obs_list:
                if image_name in obs_dict["policy"].keys():
                    # Convert from chw uint8 to hwc normalized float
                    image = torch.squeeze(obs_dict["policy"][image_name])
                    image = image.permute(2, 0, 1).clone().float()
                    image = image / 255.0
                    image = image.clip(0.0, 1.0)
                    obs[image_name] = image

        traj["obs"].append(obs)
        traj["sub_obs"].append(sub_obs)
        

        # Calculate uncertainty using the ensemble
        metrics = ensemble_uncertainty(ensemble, obs)
        # Get the std and mean action
        uncertainty = metrics['std']
        actions = metrics['mean']
        #actions = metrics['min_std_action']
        #actions = metrics['median']
        # Add the uncertainty to all joint windows
        for joint_num, unc in enumerate(uncertainty):
            windows[joint_num].append(unc)

        for joint_num in joints_to_check:
            unc_threshold = parameters[joint_num]['unc_threshold']
            peaks = sum(1 for curr_unc in windows[joint_num] if curr_unc > unc_threshold)
            if peaks > parameters[joint_num]['max_peaks']:
                unsafe_windows_detected[joint_num] += 1
        
        safety_triggered = False
        for joint_num in [0, 1, 2, 3, 4, 5, 6]:
            if unsafe_windows_detected[joint_num] > parameters[joint_num]['max_uncertain_windows']:
                # safety_triggered = safety() 
                print(f"Joint {joint_num} is uncertain at timestep {i}")
                unsafe_windows_detected[joint_num] = 0

        # Unnormalize actions
        if args_cli.norm_factor_min is not None and args_cli.norm_factor_max is not None:
            actions = (
                (actions + 1) * (args_cli.norm_factor_max - args_cli.norm_factor_min)
            ) / 2 + args_cli.norm_factor_min

        actions = actions.to(device=device).view(1, env.action_space.shape[1])
        
        # Apply actions
        obs_dict, _, terminated, truncated, _ = env.step(actions)
        obs = obs_dict["policy"]
        sub_obs = obs_dict["subtask_terms"]

        # Record trajectory
        traj["actions"].append(actions.tolist())
        traj["next_obs"].append(obs)
        traj['uncertainties'].append(uncertainty)
        traj['max_actions'].append(metrics['max'])
        traj['min_actions'].append(metrics['min'])

        # Check if rollout was successful
        if bool(success_term.func(env, **success_term.params)[0]):
            return True, traj
        elif terminated or truncated:
            return False, traj

    return False, traj


def safety():
    print(f"I'm Uncertain. {random.randint(0, 100)}")
    return True

def rollout_transformer(policy, env, success_term, horizon, device):
    """Perform a single rollout of the policy in the environment, supporting sequence-based models."""
    policy.start_episode()
    obs_dict, _ = env.reset()
    traj = dict(actions=[], obs=[], next_obs=[], sub_obs=[], uncertainties=[])

    context_length = getattr(policy, "n_obs_steps", 1)  # works for transformer or rnn models
   # print(f"Policy's expected sequence length (policy.n_obs_steps): {getattr(policy, 'n_obs_steps', 1)}")

    obs_seq = []  # list of previous obs for context window

    # Initial obs_dict from env.reset()
   # print("Initial obs_dict shapes from env.reset():")
    # for k, v in obs_dict["policy"].items():
    #     print(f"  policy['{k}']: shape={v.shape}, ndim={v.ndim}")

    # try:
    #     if hasattr(policy.policy, 'encoder') and hasattr(policy.policy.encoder, 'input_obs_group_shapes'):
    #         print("Policy's internal expected input_obs_group_shapes:")
    #         for group, keys in policy.policy.encoder.input_obs_group_shapes.items():
    #             print(f"  Group '{group}':")
    #             for k, shape in keys.items():
    #                 print(f"    Key '{k}': shape={shape}, len(shape)={len(shape)}")
    #     else:
    #         print("Could not find policy.policy.encoder.input_obs_group_shapes.")
    # except Exception as e:
    #     print(f"Error accessing policy.policy.encoder.input_obs_group_shapes: {e}")


    for i in range(horizon):
        # Prepare current observations from env for policy input
        current_policy_obs = {}
        for k, v in obs_dict["policy"].items():
            # Ensure low-dim observations are 1D (D,)
            # If env returns (1, D), squeeze the batch dimension
            if v.ndim > 1 and v.shape[0] == 1:
                processed_v = v.squeeze(0).to(device) # Apply squeeze
                # print(f"  DEBUG: Squeezed '{k}' from {v.shape} to {processed_v.shape}") # Add debug print
            else:
                processed_v = v.to(device) # No squeeze needed, assume (D,)
            current_policy_obs[k] = processed_v

        # # AFTER processing, print the shapes in current_policy_obs
        # if i == 0: # Only print for the first step to avoid spamming
        #     print("current_policy_obs shapes AFTER squeezing (if applicable):")
        #     for k, v in current_policy_obs.items():
        #         print(f"  '{k}': shape={v.shape}, ndim={v.ndim}")


        # Handle image observations specifically (if any)
        if hasattr(env.cfg, "image_obs_list"):
            for image_name in env.cfg.image_obs_list:
                if image_name in obs_dict["policy"].keys():
                    image = obs_dict["policy"][image_name].to(device)
                    # Assuming image comes as (H, W, C) and needs to be (C, H, W)
                    # If it's already (1, H, W, C), squeeze the batch dim first.
                    if image.ndim == 4 and image.shape[0] == 1:
                        image = image.squeeze(0) # Remove initial batch if present

                    # Permute and normalize after ensuring no batch dim
                    image = image.permute(2, 0, 1).clone().float() # (C, H, W)
                    image = image / 255.0
                    image = image.clip(0.0, 1.0)
                    current_policy_obs[image_name] = image


        # Add subtask terms for logging/analysis, but they are not fed to the policy
        # as per your trace (only "policy" observations are listed in the policy's encoder).
        # You had prints for these before, assuming they are okay.

        # Append to context buffer. obs_seq should hold items with shape (D,) or (C, H, W)
        traj["obs"].append(current_policy_obs) # Store the current, processed observation
        obs_seq.append(current_policy_obs) # This is what forms the sequence input

        if len(obs_seq) > context_length:
            obs_seq = obs_seq[-context_length:]

        # Pad if not enough context
        if len(obs_seq) < context_length:
            padding_obs = {}
            for k, v in obs_seq[0].items(): # Use shape of first element in obs_seq for padding
                # If obs_seq[0][k] is (D,) then zeros_like creates (D,)
                # If obs_seq[0][k] is (C,H,W) then zeros_like creates (C,H,W)
                padding_obs[k] = torch.zeros_like(v)
            obs_seq = [padding_obs] * (context_length - len(obs_seq)) + obs_seq

        # Convert context list to batched sequence dict
        seq_input = {}
        for key in obs_seq[0]:
            # Each step[key] here should be (D,) or (C, H, W)
            # torch.stack will create (context_length, D) or (context_length, C, H, W)
            # unsqueeze(0) will add the batch dimension: (1, context_length, D) or (1, context_length, C, H, W)
            seq_input[key] = torch.stack([step[key] for step in obs_seq], dim=0).to(device=device)
            #seq_input[key] = torch.stack([step[key] for step in obs_seq], dim=0).unsqueeze(0).to(device=device)

        # Print the final seq_input shapes just before policy call
        # if i == 0: # Only print for the first step
        #     print("seq_input shapes for policy just before call:")
        #     for k, v in seq_input.items():
        #         print(f"  '{k}': shape={v.shape}, ndim={v.ndim}")

        # if i == 0: # Only print for the first step
        #     print("seq_input shapes for policy just before policy(seq_input) call (LAST CHECK):")
        #     for k, v in seq_input.items():
        #         print(f"  '{k}': shape={v.shape}, ndim={v.ndim}")

        # calculate uncertainty
        # uncertainty_dict = mc_dropout_uncertainty_eval(policy=policy, obs=seq_input, niters=15)
        # traj['uncertainties'].append(uncertainty_dict['variance'])

        # Compute action from sequence
        actions = policy(seq_input)

        #print(f"DEBUG (Pre-Unnorm): Policy output 'actions' shape: {actions.shape}, ndim: {actions.ndim}, type: {type(actions)}")

        # Unnormalize actions
        if args_cli.norm_factor_min is not None and args_cli.norm_factor_max is not None:
            actions = (
                (actions + 1) * (args_cli.norm_factor_max - args_cli.norm_factor_min)
            ) / 2 + args_cli.norm_factor_min
        #print(f"DEBUG (Post-Unnorm): Policy output 'actions' shape: {actions.shape}, ndim: {actions.ndim}, type: {type(actions)}")
        # Convert policy output (torch.Tensor) to numpy array for env.step()
        # Assume actions comes as (1, Action_Dim) from policy, squeeze to (Action_Dim,)
        if isinstance(actions, torch.Tensor):
            # If it's a tensor, convert it to numpy, ensuring it's 1D (7,)
            # We already know policy returns (7,) so no squeeze is needed *here*.
            actions = actions.cpu().numpy()

        actions_tensor = torch.from_numpy(actions).to(device=device).float()
        actions_tensor = actions_tensor.unsqueeze(0) 
        # Apply actions
        #obs_dict, _, terminated, truncated, _ = env.step(actions)
        obs_dict, _, terminated, truncated, _ = env.step(actions_tensor)
        # Record trajectory - traj["next_obs"] should append the raw observation dictionary from env.step().
        traj["actions"].append(actions.tolist())
        traj["next_obs"].append(obs_dict["policy"])


        # Check if rollout was successful
        if bool(success_term.func(env, **success_term.params)[0]):
            return True, traj
        elif terminated or truncated:
            return False, traj

    return False, traj

# ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py \
# --device cuda --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --num_rollouts 50 \
# --checkpoint /logs/docs/Models/bc/model1/Isaac-Stack-Cube-Franka-IK-Rel-v0/bc_rnn_low_dim_franka_stack/20250715152224/models/model_epoch_2000.pth


# ./isaaclab.sh -p scripts/imitation_learning/robomimic/play.py --device cuda --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --num_rollouts 10 --checkpoint logs/docs/Models/bc/model2/Isaac-Stack-Cube-Franka-IK-Rel-v0/bc_rnn_low_dim_franka_stack/20250715152538/models/model_epoch_2000.pth --headless



# ./isaaclab.sh -p scripts/imitation_learning/robomimic/train.py \
# --task Isaac-Stack-Cube-Franka-IK-Rel-v0 --algo bc \
# --dataset ./docs/training_data/generated_dataset_split.hdf5 --logdir ./logs/docs/Models/bc/model8/


def load_ensemble(device, ensemble_paths):
    ensemble = []
    for path in ensemble_paths:
        policy, _ = FileUtils.policy_from_checkpoint(ckpt_path=path, device=device, verbose=True)
        ensemble.append(policy)

    return ensemble

def main():
    """Run a trained policy from robomimic with Isaac Lab environment."""
    # parse configuration
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1, use_fabric=not args_cli.disable_fabric)

    # Set observations to dictionary mode for Robomimic
    env_cfg.observations.policy.concatenate_terms = False

    #create a log handler 
    loghelper = LoggingHelper()

    # Set termination conditions
    env_cfg.terminations.time_out = None

    # Disable recorder
    env_cfg.recorders = None

    # Extract success checking function
    success_term = env_cfg.terminations.success
    env_cfg.terminations.success = None

    # Create environment
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped

    # Set seed
    torch.manual_seed(args_cli.seed)
    env.seed(args_cli.seed)

    # Acquire device
    device = TorchUtils.get_torch_device(try_to_use_cuda=True)

    # Load policy
    #policy, _ = FileUtils.policy_from_checkpoint(ckpt_path=args_cli.checkpoint, device=device, verbose=True)
    
    # load the stack cube ensemble
    stack_cube_ensemble = load_ensemble(device, ensemble_paths=[
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model0/bc_rnn_low_dim_franka_stack/20250722162848/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model1/bc_rnn_low_dim_franka_stack/20250722162851/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model2/bc_rnn_low_dim_franka_stack/20250722162852/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model3/bc_rnn_low_dim_franka_stack/20250722162853/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model4/bc_rnn_low_dim_franka_stack/20250722162853/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model5/bc_rnn_low_dim_franka_stack/20250722162854/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model6/bc_rnn_low_dim_franka_stack/20250722162855/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model7/bc_rnn_low_dim_franka_stack/20250722162855/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model8/bc_rnn_low_dim_franka_stack/20250722162856/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model9/bc_rnn_low_dim_franka_stack/20250722162857/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model10/bc_rnn_low_dim_franka_stack/20250722162857/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model11/bc_rnn_low_dim_franka_stack/20250722162857/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model12/bc_rnn_low_dim_franka_stack/20250722162857/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model13/bc_rnn_low_dim_franka_stack/20250722162857/models/model_epoch_4000.pth',
        'docs/training_data/stack_cube/uncertainty_rollout_stack_cube/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/model14/bc_rnn_low_dim_franka_stack/20250722162857/models/model_epoch_4000.pth'
    ])

    # load the pick place ensemble - 47% success rate ensemble
    # ensemble = load_ensemble(device, ensemble_paths=[ 
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model0/BC-RNN-GMM-Ensemble/20250728121319/models/model_epoch_2515_best_validation_268877.2984375.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model1/BC-RNN-GMM-Ensemble/20250728121322/models/model_epoch_1341_best_validation_264557.3390625.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model2/BC-RNN-GMM-Ensemble/20250728121323/models/model_epoch_2127_best_validation_257163.584375.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model3/BC-RNN-GMM-Ensemble/20250728121323/models/model_epoch_1466_best_validation_268943.61875.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model4/BC-RNN-GMM-Ensemble/20250728121324/models/model_epoch_1122_best_validation_259180.1421875.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model5/BC-RNN-GMM-Ensemble/20250728121324/models/model_epoch_1885_best_validation_263727.775.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model6/BC-RNN-GMM-Ensemble/20250728121324/models/model_epoch_1039_best_validation_266665.428125.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model7/BC-RNN-GMM-Ensemble/20250728121324/models/model_epoch_2821_best_validation_255741.9453125.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model8/BC-RNN-GMM-Ensemble/20250728121325/models/model_epoch_2240_best_validation_261618.5515625.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model9/BC-RNN-GMM-Ensemble/20250728121325/models/model_epoch_2576_best_validation_266381.5921875.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model10/BC-RNN-GMM-Ensemble/20250728121325/models/model_epoch_1710_best_validation_254380.1484375.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model11/BC-RNN-GMM-Ensemble/20250728121326/models/model_epoch_2149_best_validation_252307.69375.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model12/BC-RNN-GMM-Ensemble/20250728121326/models/model_epoch_1114_best_validation_266930.321875.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model13/BC-RNN-GMM-Ensemble/20250728121327/models/model_epoch_2771_best_validation_268996.6671875.pth',
    #     'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model14/BC-RNN-GMM-Ensemble/20250728121327/models/model_epoch_2885_best_validation_250146.475.pth',
    # ])

    # load the pick place ensemble - 65% success rate ensemble
    pick_place_ensemble = load_ensemble(device, ensemble_paths=[ 
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model0/BC-RNN-GMM-Ensemble/20250728121319/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model1/BC-RNN-GMM-Ensemble/20250728121322/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model2/BC-RNN-GMM-Ensemble/20250728121323/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model3/BC-RNN-GMM-Ensemble/20250728121323/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model4/BC-RNN-GMM-Ensemble/20250728121324/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model5/BC-RNN-GMM-Ensemble/20250728121324/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model6/BC-RNN-GMM-Ensemble/20250728121324/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model7/BC-RNN-GMM-Ensemble/20250728121324/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model8/BC-RNN-GMM-Ensemble/20250728121325/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model9/BC-RNN-GMM-Ensemble/20250728121325/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model10/BC-RNN-GMM-Ensemble/20250728121325/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model11/BC-RNN-GMM-Ensemble/20250728121326/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model12/BC-RNN-GMM-Ensemble/20250728121326/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model13/BC-RNN-GMM-Ensemble/20250728121327/models/model_epoch_3000.pth',
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model14/BC-RNN-GMM-Ensemble/20250728121327/models/model_epoch_3000.pth',
    ])

    single = load_ensemble(device, ensemble_paths=[ 
        'docs/training_data/pick_place/uncertainty_rollout_pick_place/ensemble/model0/BC-RNN-GMM-Ensemble/20250728121319/models/model_epoch_3000.pth'
        for _ in range(15)
    ])

    # load the pick place ensemble - 60% success rate ensemble
    # ensemble = load_ensemble(device, ensemble_paths=[ 
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    #     'docs/model_epoch_2748_best_validation_273167.9078125.pth',
    # ])

    # 'unc_threshold': 0.1,
    # 'max_peaks': 20,
    # 'window_size': 40
    parameters = {
        'stack_cube': {
             6: {
                'unc_threshold': 0.05,
                'max_peaks': 15,
                'window_size': 30,
                'max_uncertain_windows': 3
            },
            5: {
                'unc_threshold': 0.06,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },

            4: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            3: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            2: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            1: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            0: {
                'unc_threshold': 0.04,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            }
        },
        
        'pick_place': {
            6: {
                'unc_threshold': 0.05,
                'max_peaks': 15,
                'window_size': 30,
                'max_uncertain_windows': 3
            },
            5: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 1
            },

            4: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            3: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            2: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            1: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            },
            0: {
                'unc_threshold': 0.02,
                'max_peaks': 2,
                'window_size': 10,
                'max_uncertain_windows': 3
            }

        }
    }

    task = "stack_cube" # stack_cube or pick_place
    model_name = f"ensemble"
    #uncertainties_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/ensemble/Isaac-Stack-Cube-Franka-IK-Rel-v0/{model_name}/uncertainties.txt"
    # ensemble is at uncertainties7.txt and single is at uncertainties8.txt
    number = '14'
    uncertainties_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/uncertainties{number}.txt"
    actions_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/actions{number}.txt"
    min_actions_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/min_actions{number}.txt"
    max_actions_path = f"./docs/training_data/{task}/uncertainty_rollout_{task}/{model_name}/max_actions{number}.txt"
    rollout_log_path = f"{uncertainties_path[:len(uncertainties_path)-4]}_rollout_log.txt"
    
    try:
        os.remove(rollout_log_path)
    except FileNotFoundError as filenotfounderror:
        pass
    
    # clear uncertainties file
    with open(uncertainties_path, 'w') as file:
        pass
    with open(actions_path, 'w') as file:
        pass
    with open(min_actions_path, 'w') as file:
        pass
    with open(max_actions_path, 'w') as file:
        pass
    # clear rollout logger file
    with open(loghelper.namefile, 'w') as file:
        pass
    
    # Run policy
    results = []
    for trial in range(args_cli.num_rollouts):
        print(f"[INFO] Starting trial {trial}")
       # loghelper.startEpoch(trial)
        #terminated, traj = rollout(policy, env, success_term, args_cli.horizon, device)
        #terminated, traj = rollout_transformer(policy, env, success_term, args_cli.horizon, device)
        terminated, traj = rollout_ensemble(stack_cube_ensemble, env, success_term, args_cli.horizon, device, parameters[task])
        # save the uncertainties
        with open(uncertainties_path, 'a') as file:
            for i, var in enumerate(traj['uncertainties']):
                line = " ".join([str(v.item()) for v in var]) + f" {terminated}\n"
                file.write(f"{str(i)} {line}")
        # save the actions
        with open(actions_path, 'a') as file:
            for i, var in enumerate(traj['actions']):
                line = " ".join([str(v) for v in var[0]]) + f" {terminated}\n"
                file.write(f"{str(i)} {line}")
        # save the max actions
        with open(max_actions_path, 'a') as file:
            for i, var in enumerate(traj['max_actions']):
                line = " ".join([str(v.item()) for v in var[0]]) + f" {terminated}\n"
                file.write(f"{str(i)} {line}")
        # save the min actions
        with open(min_actions_path, 'a') as file:
            for i, var in enumerate(traj['min_actions']):
                line = " ".join([str(v.item()) for v in var[0]]) + f" {terminated}\n"
                file.write(f"{str(i)} {line}")
            
        results.append(terminated)
        print(f"[INFO] Trial {trial}: {terminated}\n")
        #loghelper.stopEpoch(trial)

    print(f"\nSuccessful trials: {results.count(True)}, out of {len(results)} trials")
    print(f"Success rate: {results.count(True) / len(results)}")
    print(f"Trial Results: {results}\n")


    env.close()



if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
