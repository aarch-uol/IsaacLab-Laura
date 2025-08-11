#region Header
# IsaacLab — Utilities Module
# --------------------------------------------------------------------------------
# File: utils.py
# Author: Harry WF Cheung (University of Liverpool)
# Date: 26/06/2025
# Description: Shared utilities for demonstration workflows in IsaacLab.
#              Includes argument parsing, recorder configs, callbacks, logging, 
#              rate limiting, teleop device interfaces, and action preprocessing.
# --------------------------------------------------------------------------------
# License: BSD-3-Clause (© 2022–2025 Isaac Lab Project Developers)
#endregion

# --------------------------------------------------------------------------------
# region import

from isaaclab.devices import OpenXRDevice, Se3Keyboard, Se3SpaceMouse, Se3Gamepad
from isaaclab.devices.openxr.retargeters.manipulator import GripperRetargeter, Se3AbsRetargeter, Se3RelRetargeter
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg
from isaaclab.managers import RecorderTerm, RecorderTermCfg
from isaaclab.utils.math import combine_frame_transforms
from isaaclab.utils import configclass
import gymnasium as gym
import numpy as np
import torch
import time

"""
try:
    from isaaclab.harrys_mods.utils import DEBUG   # uses False if module exists
except ImportError:
    DEBUG = True    

if DEBUG == True:
"""

DEBUG = False

# --------------------------------------------------------------------------------
# region Logger

def log(env, step):
    command = env.command_manager.get_command("object_pose")
    robot = env.scene["robot"]
    object = env.scene["object"]
    ee = env.scene["ee_frame"]
    
    ee_pos = ee.data.target_pos_w[0, 0].cpu().numpy()
    obj_pos = object.data.root_pos_w[0].cpu().numpy()

    des_pos_b = command[:, :3]
    des_pos_w, _ = combine_frame_transforms(
        robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b
    )
    goal_pos = des_pos_w[0].cpu().numpy()
    differece = (obj_pos - goal_pos)

    dist_obj_to_goal = float(np.linalg.norm(obj_pos - goal_pos))
    dist_ee_to_obj = float(np.linalg.norm(ee_pos - obj_pos))
    
    # Print reward/subtask table
    print(f"""  
        +-----------------------------------------------------+
        |            Step {step:4d} - Term Monitor            
        +------------------------+----------------------------+
        | End-Effector Position : {ee_pos.round(4)}           
        | Object Position       : {obj_pos.round(4)}           
        | Goal Position (W)     : {goal_pos.round(4)}
        | Difference            : {differece.round(4)}
        | Dist(Object → Goal)   : {dist_obj_to_goal:.4f} 
        | Dist(EE → Object)     : {dist_ee_to_obj:.4f} 
        +-----------------------------------------------------+
        """)

# --------------------------------------------------------------------------------
# region Playback Control

def play_cb():
    global is_paused
    is_paused = False


def pause_cb():
    global is_paused
    is_paused = True


def skip_episode_cb():
    global skip_episode
    skip_episode = True


def mark_subtask_cb():
    global current_action_index, marked_subtask_action_indices
    marked_subtask_action_indices.append(current_action_index)
    print(f"Marked a subtask signal at action index: {current_action_index}")

# --------------------------------------------------------------------------------
# region Rate Limiter

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

# --------------------------------------------------------------------------------
# region Mimic data

class PreStepDatagenInfoRecorder(RecorderTerm):
    """Recorder term that records the datagen info data in each step."""

    def record_pre_step(self):
        eef_pose_dict = {}
        for eef_name in self._env.cfg.subtask_configs.keys():
            eef_pose_dict[eef_name] = self._env.get_robot_eef_pose(eef_name=eef_name)

        datagen_info = {
            "object_pose": self._env.get_object_poses(),
            "eef_pose": eef_pose_dict,
            "target_eef_pose": self._env.action_to_target_eef_pose(self._env.action_manager.action),
        }
        return "obs/datagen_info", datagen_info


@configclass
class PreStepDatagenInfoRecorderCfg(RecorderTermCfg):
    """Configuration for the datagen info recorder term."""

    class_type: type[RecorderTerm] = PreStepDatagenInfoRecorder


class PreStepSubtaskTermsObservationsRecorder(RecorderTerm):
    """Recorder term that records the subtask completion observations in each step."""

    def record_pre_step(self):
        return "obs/datagen_info/subtask_term_signals", self._env.get_subtask_term_signals()


@configclass
class PreStepSubtaskTermsObservationsRecorderCfg(RecorderTermCfg):
    """Configuration for the step subtask terms observation recorder term."""

    class_type: type[RecorderTerm] = PreStepSubtaskTermsObservationsRecorder


@configclass
class MimicRecorderManagerCfg(ActionStateRecorderManagerCfg):
    """Mimic-specific recorder manager config with datagen info and subtask term recorders."""

    record_pre_step_datagen_info = PreStepDatagenInfoRecorderCfg()
    record_pre_step_subtask_term_signals = PreStepSubtaskTermsObservationsRecorderCfg()

# --------------------------------------------------------------------------------
# region Action Processor

def pre_process_actions(
    teleop_data: tuple | list,
    num_envs: int,
    device: str,
    task: str,
) -> torch.Tensor:
    """
    Convert teleop data to the format expected by the environment action space.

    Args:
        teleop_data: Data from the teleoperation device.
        num_envs: Number of environments.
        device: Device to create tensors on.
        task: Name of the task (to determine action format).

    Returns:
        Processed actions as a tensor.
    """
    if "Reach" in task:
        delta_pose, gripper_command = teleop_data
        delta_pose = torch.tensor(delta_pose, dtype=torch.float, device=device).repeat(num_envs, 1)
        return delta_pose

    elif "PickPlace-GR1T2" in task:
        # Expect teleop_data[0] to be a tuple of (left_wrist_pose, right_wrist_pose, hand_joints)
        (left_wrist_pose, right_wrist_pose, hand_joints) = teleop_data[0]
        actions = torch.tensor(
            np.concatenate([left_wrist_pose, right_wrist_pose, hand_joints]),
            device=device,
            dtype=torch.float32,
        ).unsqueeze(0)
        return actions

    else:
        delta_pose, gripper_command = teleop_data
        delta_pose = torch.tensor(delta_pose, dtype=torch.float, device=device).repeat(num_envs, 1)
        gripper_vel = torch.full((delta_pose.shape[0], 1), float(-1 if gripper_command else 1), device=device)
        return torch.cat([delta_pose, gripper_vel], dim=1)

# --------------------------------------------------------------------------------
# region Device Handler

def load_device(device: str, sensitivity: float = 1.0):
    """
    Factory for SE(3) teleoperation interfaces.
    Args:
        device (str): Teleop device name (e.g., "keyboard", "spacemouse", "gamepad", etc.)
        sensitivity (float): Sensitivity multiplier.

    Returns:
        TeleopInterface instance.
    """
    supported_devices = {
        "keyboard": (Se3Keyboard, 0.05),
        "spacemouse": (Se3SpaceMouse, 0.2),
        "gamepad": (Se3Gamepad, 0.1),
    }
    device = device.lower()
    if device not in supported_devices:
        raise ValueError(f"Device:'{device}' invalid. Supported: {list(supported_devices.keys())}")
    device_cls, base_val = supported_devices[device]
    return device_cls(pos_sensitivity=base_val * sensitivity, rot_sensitivity=base_val * sensitivity)

def make_callback(callbacks, teleop_interface):
    for key, func in callbacks.items():
        teleop_interface.add_callback(key, func)


def create_teleop_device(device_name: str, env, env_cfg, callbacks: dict[str, callable] = None):
    """
    Create and configure teleoperation device for robot control.

    Args:
        device_name: Device name, e.g., 'keyboard', 'spacemouse', 'handtracking', etc.
        env: Isaac Lab Gym environment.
        env_cfg: Environment config used for OpenXR setup.
        callbacks: Optional dictionary of callbacks like {"RESET": func, "START": func, "STOP": func}.

    Returns:
        Teleop interface instance.
    """


    callbacks = callbacks or {}
    device_name = device_name.lower()

    if device_name in ["keyboard", "spacemouse", "gamepad"]:
        teleop_interface = load_device(device_name)

    elif "dualhandtracking_abs" in device_name and "GR1T2" in env.cfg.env_name:
        from isaacsim.xr.openxr import OpenXRSpec
        from isaaclab.devices.openxr.retargeters.humanoid.fourier.gr1t2_retargeter import GR1T2Retargeter
        gr1t2_retargeter = GR1T2Retargeter(
            enable_visualization=True,
            num_open_xr_hand_joints=2 * (int(OpenXRSpec.HandJointEXT.XR_HAND_JOINT_LITTLE_TIP_EXT) + 1),
            device=env.unwrapped.device,
            hand_joint_names=env.scene["robot"].data.joint_names[-22:],
        )
        teleop_interface = OpenXRDevice(env_cfg.xr, retargeters=[gr1t2_retargeter])
        make_callback(callbacks, teleop_interface)

    elif "handtracking" in device_name:
        if "_abs" in device_name:
            ee_retargeter = Se3AbsRetargeter(
                bound_hand=OpenXRDevice.TrackingTarget.HAND_RIGHT,
                zero_out_xy_rotation=True,
            )
        else:
            ee_retargeter = Se3RelRetargeter(
                bound_hand=OpenXRDevice.TrackingTarget.HAND_RIGHT,
                zero_out_xy_rotation=True,
            )
        grip_retargeter = GripperRetargeter(bound_hand=OpenXRDevice.TrackingTarget.HAND_RIGHT)
        teleop_interface = OpenXRDevice(env_cfg.xr, retargeters=[ee_retargeter, grip_retargeter])
        make_callback(callbacks, teleop_interface)


    else:
        raise ValueError(
            f"Invalid device interface '{device_name}'. Supported: 'keyboard', 'spacemouse', 'gamepad', "
            "'handtracking', 'handtracking_abs', 'dualhandtracking_abs'."
        )

    # Attach universal callbacks

    return teleop_interface
