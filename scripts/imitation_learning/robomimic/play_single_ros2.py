# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play and evaluate a trained policy from robomimic.

This script loads a robomimic policy and plays it in an Isaac Lab environment.
It has been modified to publish robot actions to a ROS2 topic.

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
parser.add_argument("--horizon", type=int, default=400, help="Step horizon of each rollout.")
parser.add_argument("--num_rollouts", type=int, default=1, help="Number of rollouts.")
parser.add_argument("--seed", type=int, default=101, help="Random seed.")
parser.add_argument(
    "--norm_factor_min", type=float, default=None, help="Optional: minimum value of the normalization factor."
)
parser.add_argument(
    "--norm_factor_max", type=float, default=None, help="Optional: maximum value of the normalization factor."
)
parser.add_argument("--enable_pinocchio", default=False, action="store_true", help="Enable Pinocchio.")


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

import copy
import gymnasium as gym
import torch

import robomimic.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils

if args_cli.enable_pinocchio:
    import isaaclab_tasks.manager_based.manipulation.pick_place  # noqa: F401

from isaaclab_tasks.utils import parse_env_cfg
from evaluation import inject_dropout_layers, MC_dropout_uncertainty, remove_dropout_layers
from isaaclab.utils.pretty_printer import print_table, LogRollout
from isaaclab.utils import TrajectoryLogger
from isaaclab.safety.safety_logic import SafetyLogic

# --- ROS2 imports ---
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class PolicyNode(Node):
    def __init__(self):
        super().__init__('isaaclab_policy_node')
        self.action_publisher = self.create_publisher(Float32MultiArray, '/policy_actions', 10)

        self.real_position_sub = self.create_subscription(
            JointState,
            '/franka_robot_state_broadcaster/measured_joint_states',
            self.real_position_callback,
            10
        )

        self.latest_real_position = None

    def real_position_callback(self, msg: JointState):
        """Store the latest JointState as a dict for easy use"""
        self.latest_real_position = {
            "names": list(msg.name),
            "positions": list(msg.position),
            "velocities": list(msg.velocity),
            "efforts": list(msg.effort),
            "stamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id
        }
        self.get_logger().info(
            f"Real position received: positions={self.latest_real_position['positions']}"
        )

class RobotStateMessage:
    joint_pos : list[float]
    gripper_pos : list[float]
    sm_state : int
    time : int


def rollout(policy, env, success_term, horizon, device, logging, traj_logging, node):
    """Perform a single rollout of the policy in the environment.

    Args:
        policy: The robomimicpolicy to play.
        env: The environment to play in.
        horizon: The step horizon of each rollout.
        device: The device to run the policy on.
        node: The ROS2 node object.
        action_publisher: ROS2 publisher for robot actions.

    Returns:
        terminated: Whether the rollout terminated.
        traj: The trajectory of the rollout.
    """

    policy.start_episode()
    obs_dict, _ = env.reset()
    traj = dict(actions=[], obs=[], next_obs=[], sub_obs=[], uncertainties=[], obstacles =[])

    node.get_logger().info("Waiting for /real_position message...")
    while rclpy.ok() and node.latest_real_position is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    rp = node.latest_real_position
    node.get_logger().info(f"Using initial real joint positions: {rp['positions']}")
    
    
    safety_logic = SafetyLogic(obs_dict["policy"]["obstacle_pos"], 0.3)
    
    # try:
    #     print("obstacle initial state: ", env.cfg.scene.obstacle.init_state)
    # except:
    #     print("Env cfg not found ", )

    for i in range(horizon):
        # Prepare observations
        rclpy.spin_once(node, timeout_sec=0.0)   # process subscriber callbacks
        rp = node.latest_real_position
        if rp is not None:
            print(f"Got message : {rp['positions']}")
            # if "joint_pos" in obs_dict["policy"]:
            #     obs_dict["policy"]["joint_pos"] = torch.tensor(
            #         rp["positions"], dtype=torch.float32
            #     ).unsqueeze(0)
            # if "joint_vel" in obs_dict["policy"] and len(rp["velocities"]) > 0:
            #     obs_dict["policy"]["joint_vel"] = torch.tensor(
            #         rp["velocities"], dtype=torch.float32
            #     ).unsqueeze(0)
        obs = copy.deepcopy(obs_dict["policy"])
        sub_obs = copy.deepcopy(obs_dict["subtask_terms"])
        for ob in obs:
            obs[ob] = torch.squeeze(obs[ob])
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
        # Compute actions
        actions = policy(obs)

        # Unnormalize actions
        if args_cli.norm_factor_min is not None and args_cli.norm_factor_max is not None:
            actions = (
                (actions + 1) * (args_cli.norm_factor_max - args_cli.norm_factor_min)
            ) / 2 + args_cli.norm_factor_min

        uncertainty_dict = safety_logic._MC_dropout_uncertainty(policy, obs)
        traj['uncertainties'].append(uncertainty_dict['variance'])

        
        actions = torch.from_numpy(actions).to(device=device).view(1, env.action_space.shape[1])

        # publish the action to ros2 topic
        # --- Publish to ROS2 ---

        robot_state_message  = RobotStateMessage()
        robot_state_message.joint_pos = actions[0].tolist()
        node.action_publisher.publish(action_msg)
        node.get_logger().info(f"Published actions: {action_msg.data}")
       # print("ROS2 spin complete")

        # Apply actions
        obs_dict, _, terminated, truncated, _ = env.step(actions)
        obs = obs_dict["policy"]

        # Record trajectory
        traj["actions"].append(actions.tolist())
        traj["next_obs"].append(obs)

        ######## EVAL #######
        
        collision_exp, dist = safety_logic.exp_static_coll(obs["eef_pos"])
      
        #  print_table(["Step", "Variance", "Collision dist", "Safety violation"], [i, uncertainty_dict['variance'].data[-1], dist, collision_exp])
        logging.write_to_log([i, obs["joint_pos"][-1][-2].item() ,uncertainty_dict['variance'].data[-1], dist, collision_exp])
      
        traj_logging.add_data(i, obs["eef_pos"][0], obs["joint_vel"][0][-1])
        # Check if rollout was successful
        if bool(success_term.func(env, **success_term.params)[0]):
            return True, traj
        elif terminated or truncated:
            return False, traj

    return False, traj


def main():
    """Run a trained policy from robomimic with Isaac Lab environment."""
    
    # --- NEW: Initialize ROS2 and create a publisher ---
    print("Initializing ROS2...")
    rclpy.init(args=None)
    # Create a ROS2 node for the simulation script
    node = PolicyNode()
    
    # parse configuration
    env_cfg = parse_env_cfg(args_cli.task, device=args_cli.device, num_envs=1, use_fabric=not args_cli.disable_fabric)

    # Set observations to dictionary mode for Robomimic
    env_cfg.observations.policy.concatenate_terms = False
   
    # Set termination conditions
    env_cfg.terminations.time_out = None

    # Disable recorder
    env_cfg.recorders = None

    # Extract success checking function
    success_term = env_cfg.terminations.success
    env_cfg.terminations.success = None

    # Create environment
    env = gym.make(args_cli.task, cfg=env_cfg).unwrapped
   
    # print("scene : ", env.cfg.scene.obstacle)
    # Set seed
    torch.manual_seed(args_cli.seed)
    env.seed(args_cli.seed)

    # Acquire device
    device = TorchUtils.get_torch_device(try_to_use_cuda=True)

    # Load policy
    policy, _ = FileUtils.policy_from_checkpoint(ckpt_path=args_cli.checkpoint, device=device, verbose=True)
    print("policy : ", type(policy), " value : " ,  policy)

    # Run policy
    results = []
    for trial in range(args_cli.num_rollouts):
        filename = "BC_RNN_TEST_RUN_" + str(trial)
        logging = LogRollout(filename,["Step", "Gripper pose" , "Variance", "Collision dist", "Safety violation"] )
        traj_logging = TrajectoryLogger(filename)
        print(f"[INFO] Starting trial {trial}")
        logging.new_trial(trial)
        # --- NEW: Pass the action_publisher and node to the rollout function ---
        terminated, traj = rollout(policy, env, success_term, args_cli.horizon, device, logging, traj_logging, node)
        results.append(terminated)
        logging.trial_outcome(terminated)
        print(f"[INFO] Trial {trial}: {terminated}\n")

    print(f"\nSuccessful trials: {results.count(True)}, out of {len(results)} trials")
    print(f"Success rate: {results.count(True) / len(results)}")
    print(f"Trial Results: {results}\n")

    env.close()
    
    # --- NEW: Clean up ROS2 resources ---
    node.destroy_node()
    rclpy.shutdown()
    print("ROS2 shutdown complete.")


if __name__ == "__main__":
    main()
    
    # close sim app
    simulation_app.close()
