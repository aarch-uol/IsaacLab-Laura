# run this script: ./isaaclab.sh -p scripts/imitation_learning/robomimic//evaluation.py --checkpoint ./docs/Models/model_epoch_5000.pth

import argparse
import copy
import gymnasium as gym

import torch
import torch.nn as nn
import torch.nn.functional as F

import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils



# add argparse arguments
parser = argparse.ArgumentParser(description="Evaluate robomimic policy for Isaac Lab environment.")
parser.add_argument("--checkpoint", type=str, default=None, help="Pytorch model checkpoint to load.")
parser.add_argument("--seed", type=int, default=101, help="Random seed.")

args_cli = parser.parse_args()


def MC_dropout_uncertainty(policy, obs, niters=50):
    """
    Low std: Model is confident for that output dimension (joint position).

    High std: Model is unsure - often in areas outside training distribution or in ambiguous states.
    """
    actions = [torch.from_numpy(policy(obs)) for i in range(niters)]
    actions = torch.stack(actions)
    #mean = torch.mean(actions, dim=0)
    std = torch.std(actions, dim=0)

    return std



def inject_dropout_layers(policy, probability=0.5):
    modules = list(policy.policy.nets['policy'].named_modules())
    #print(f"Total modules found: {len(modules)}")

    def dropout_hook(module, _, output):
        #print("applying dropout")
        return F.dropout(output, p=probability, training=True) # setting training=True forces dropout to happen regardless of whether the model is in train or evaluation mode

    hooks = []
    for name, module in modules:
        if isinstance(module, nn.Linear):
            hook = module.register_forward_hook(dropout_hook)
            hooks.append(hook)
    
    

device = TorchUtils.get_torch_device(try_to_use_cuda=True)

policy, _ = FileUtils.policy_from_checkpoint(ckpt_path=args_cli.checkpoint, device=device, verbose=True)

inject_dropout_layers(policy=policy, probability=0.5)

obs_dict = {'policy': {
            'joint_pos': torch.tensor([[0., 0., 0., 0., 0., 0., 0., 0., 0.]], device=device), 
            'joint_vel': torch.tensor([[0., 0., 0., 0., 0., 0., 0., 0., 0.]], device=device), 
            'object_position': torch.tensor([[0.4975, 0.0483, 0.0203]], device=device), 
            'target_object_position': torch.tensor([[ 0.4000, -0.3000,  0.0800,  1.0000,  0.0000,  0.0000,  0.0000]], device=device), 
            'actions': torch.tensor([[0., 0., 0., 0., 0., 0., 0.]], device=device), 
            'object_to_target': torch.tensor([False], device=device), 
            'eef_pos': torch.tensor([[ 0.3764, -0.0050,  0.0891]], device=device), 
            'eef_quat': torch.tensor([[ 0.6433,  0.0366,  0.7636, -0.0415]], device=device), 
            'gripper_pos': torch.tensor([[ 0.0400, -0.0400]], device=device)
        }, 
    'subtask_terms': {
            'appr': torch.tensor([False], device=device), 
            'grasp': torch.tensor([False], device=device), 
            'lift': torch.tensor([False], device=device), 
            'appr_goal': torch.tensor([False], device=device)
        }
    }


policy.start_episode()
#policy.policy.nets['policy'].eval()

# Prepare observations
obs = copy.deepcopy(obs_dict["policy"])
sub_obs = copy.deepcopy(obs_dict["subtask_terms"])

for ob in obs:
    obs[ob] = torch.squeeze(obs[ob])
    #print(f"found observation : {obs[ob]}")

for subob in sub_obs:
    sub_obs[subob] = torch.squeeze(sub_obs[subob])

import time

start_time = time.time()
std = MC_dropout_uncertainty(policy=policy, obs=obs, niters=50)
end_time = time.time()

print(f"tds: {std}\ntime taken: {end_time - start_time}")
#action = policy(obs)
#print("Action:", action)



