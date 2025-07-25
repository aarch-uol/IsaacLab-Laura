# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject, RigidObjectCollection
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import FrameTransformer
# from isaaclab.utils.logging_helper import LoggingHelper, ErrorType, LogType
from isaaclab.utils.math import subtract_frame_transforms, combine_frame_transforms
import math

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def object_positions_in_world_frame(
    env: ManagerBasedRLEnv,
    object_1_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    object_2_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
) -> torch.Tensor:
    """The position of the cubes in the world frame."""
    object_1: RigidObject = env.scene[object_1_cfg.name]
    object_2: RigidObject = env.scene[object_2_cfg.name]

    return torch.cat((object_1.data.root_pos_w, object_2.data.root_pos_w), dim=1) 


def object_orientations_in_world_frame(
    env: ManagerBasedRLEnv,
    object_1_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    object_2_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
):
    """The orientation of the cubes in the world frame."""
    object_1: RigidObject = env.scene[object_1_cfg.name]
    object_2: RigidObject = env.scene[object_2_cfg.name]

    return torch.cat((object_1.data.root_quat_w, object_2.data.root_quat_w), dim=1) 


def object_obs(
    env: ManagerBasedRLEnv,
    object_1_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    object_2_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
):
    """
    Object observations (in world frame):
        object_1 pos,
        object_1 quat,
        object_2 pos,
        object_2 quat,
        gripper to object_1,
        gripper to object_2,
        object_1 to object_2,
    """
    object_1: RigidObject = env.scene[object_1_cfg.name]
    object_2: RigidObject = env.scene[object_2_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    object_1_pos_w = object_1.data.root_pos_w
    object_1_quat_w = object_1.data.root_quat_w

    object_2_pos_w = object_2.data.root_pos_w
    object_2_quat_w = object_2.data.root_quat_w


    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]
    gripper_to_object_1 = object_1_pos_w - ee_pos_w
    gripper_to_object_2 = object_2_pos_w - ee_pos_w

    object_1_to_2 = object_1_pos_w - object_2_pos_w

    return torch.cat(
        (
            object_1_pos_w - env.scene.env_origins,
            object_1_quat_w,
            object_2_pos_w - env.scene.env_origins,
            object_2_quat_w,
            gripper_to_object_1,
            gripper_to_object_2,
            object_1_to_2,
        ),
        dim=1,
    )


def ee_frame_pos(env: ManagerBasedRLEnv, ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame")) -> torch.Tensor:
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_frame_pos = ee_frame.data.target_pos_w[:, 0, :] - env.scene.env_origins[:, 0:3]

    return ee_frame_pos


def ee_frame_quat(env: ManagerBasedRLEnv, ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame")) -> torch.Tensor:
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    ee_frame_quat = ee_frame.data.target_quat_w[:, 0, :]

    return ee_frame_quat


def gripper_pos(env: ManagerBasedRLEnv, robot_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
    robot: Articulation = env.scene[robot_cfg.name]
    finger_joint_1 = robot.data.joint_pos[:, -1].clone().unsqueeze(1)
    finger_joint_2 = -1 * robot.data.joint_pos[:, -2].clone().unsqueeze(1)

    return torch.cat((finger_joint_1, finger_joint_2), dim=1)

def reach_object(
    env: ManagerBasedRLEnv,
    threshold: float = 0.05,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Reward the agent for reaching the object using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    object: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    # Target object position: (num_envs, 3)
    object_pos_w = object.data.root_pos_w
    # End-effector position: (num_envs, 3)
    ee_w = ee_frame.data.target_pos_w[..., 0, :]
    # Distance of the end-effector to the object: (num_envs,)
    object_ee_distance = torch.norm(object_pos_w - ee_w, dim=1)
    # if object_ee_distance.item() < std :
    #     loghelper.logsubtask(LogType.APPR)
    return object_ee_distance < threshold


def object_grasped(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg,
    object_cfg: SceneEntityCfg,
    diff_threshold: float = 0.06,
    gripper_open_val: torch.tensor = torch.tensor([0.04]),
    gripper_threshold: float = 0.005,
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Check if an object is grasped by the specified robot."""

    robot: Articulation = env.scene[robot_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]

    object_pos = object.data.root_pos_w
    end_effector_pos = ee_frame.data.target_pos_w[:, 0, :]
    pose_diff = torch.linalg.vector_norm(object_pos - end_effector_pos, dim=1)

    grasped = torch.logical_and(
        pose_diff < diff_threshold,
        torch.abs(robot.data.joint_pos[:, -1] - gripper_open_val.to(env.device)) > gripper_threshold,
    )
    grasped = torch.logical_and(
        grasped, torch.abs(robot.data.joint_pos[:, -2] - gripper_open_val.to(env.device)) > gripper_threshold
    )
    # if grasped[0]:
    #     loghelper.logsubtask(LogType.GRASP)

    return grasped

def is_object_lifted(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    threshold : float = 0.05,
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    #return true when object z coord above a threshold value 
    object = env.scene[object_cfg.name]
    # if object.data.root_pos_w[:, 2].item() > threshold : 
    #     loghelper.logsubtask(LogType.LIFT)
 
    return object.data.root_pos_w[:, 2] > threshold

### Add a waypoint here

def reach_object2(
    env: ManagerBasedRLEnv,
    threshold: float = 0.1,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Reward the agent for reaching the object using tanh-kernel."""
    # extract the used quantities (to enable type-hinting)
    ### Currently not working
    object2: RigidObject = env.scene[object_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    # Target object2 position: (num_envs, 3)
    object_pos_w = object2.data.root_pos_w
    # End-effector position: (num_envs, 3)
    ee_w = ee_frame.data.target_pos_w[..., 0, :]
    # Distance of the end-effector to the object2: (num_envs,)
    object2_ee_distance = torch.norm(object_pos_w - ee_w, dim=1)
    # if object2_ee_distance.item() < std :
    #     loghelper.logsubtask(LogType.APPR_2)
    return object2_ee_distance < threshold

def pour_object(
    env: ManagerBasedEnv,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    angle_threshold: int = 45, 
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """The agent is pouring into something else"""
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    angle = torch.tensor(angle_threshold)
    ee_w = ee_frame.data.target_quat_w[..., 0, :]
    # Normalize quaternion
    norms = torch.linalg.norm(ee_w, dim=1, keepdim=True)
    quats_norm = ee_w / norms
    # Extract w component (scalar part - directly relates to angle)
    w = quats_norm[:, 3]
    # Clamp w to valid range [-1, 1] to avoid NaNs in arccos
    w_clamped = torch.clamp(w, -1.0, 1.0)
    # Calculate rotation angle theta = 2 * arccos(w)
    angles_rad = 2.0 * torch.acos(w_clamped)
    # Optional: convert radians to degrees
    angles_deg = angles_rad * (180.0 / torch.pi)
    angles_deg = 270 - angles_deg
    return angles_deg < angle

def reorient_object(
    env: ManagerBasedEnv,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    angle_threshold: int = 175, 
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """The agent is reorienting the object"""
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]
    angle = torch.tensor(angle_threshold)
    ee_w = ee_frame.data.target_quat_w[..., 0, :]
    # Normalize quaternion
    norms = torch.linalg.norm(ee_w, dim=1, keepdim=True)
    quats_norm = ee_w / norms
    # Extract w component (scalar part - directly relates to angle)
    w = quats_norm[:, 3]
    # Clamp w to valid range [-1, 1] to avoid NaNs in arccos
    w_clamped = torch.clamp(w, -1.0, 1.0)
    # Calculate rotation angle theta = 2 * arccos(w)
    angles_rad = 2.0 * torch.acos(w_clamped)
    # Optional: convert radians to degrees
    angles_deg = angles_rad * (180.0 / torch.pi)
    angles_deg = 360 - angles_deg
    return angles_deg > angle

def object_stacked(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg,
    # Upper and lower object cfg defined in stack_env_cfg when stacked in subtask
    upper_object_cfg: SceneEntityCfg,
    lower_object_cfg: SceneEntityCfg,
    command_name: str = "object_pose",
    xy_threshold: float = 0.05,
    height_threshold: float = 0.005,
    height_diff: float = 0.0468,
    gripper_open_val: torch.tensor = torch.tensor([0.04]),
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Check if an object is stacked by the specified robot."""

    robot: Articulation = env.scene[robot_cfg.name]
    upper_object: RigidObject = env.scene[upper_object_cfg.name]
    lower_object: RigidObject = env.scene[lower_object_cfg.name]
    command = env.command_manager.get_command(command_name)

    pos_diff = upper_object.data.root_pos_w - lower_object.data.root_pos_w
    height_dist = torch.linalg.vector_norm(pos_diff[:, 2:], dim=1)
    xy_dist = torch.linalg.vector_norm(pos_diff[:, :2], dim=1)

    stacked = torch.logical_and(xy_dist < xy_threshold, (height_dist - height_diff) < height_threshold)

    # Checking right and left gripper positions
    stacked = torch.logical_and(
        torch.isclose(robot.data.joint_pos[:, -1], gripper_open_val.to(env.device), atol=1e-4, rtol=1e-4), stacked
    )
    stacked = torch.logical_and(
        torch.isclose(robot.data.joint_pos[:, -2], gripper_open_val.to(env.device), atol=1e-4, rtol=1e-4), stacked
    )
    # if stacked[0]:
    #     loghelper.logsubtask(LogType.STACK)

    return stacked

def object_near_goal(
    env: ManagerBasedRLEnv,
    command_name: str = "object_pose",
    threshold: float = 0.02,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Termination condition for the object reaching the goal position.

    Args:
        env: The environment.
        command_name: The name of the command that is used to control the object.
        threshold: The threshold for the object to reach the goal position. Defaults to 0.02.
        robot_cfg: The robot configuration. Defaults to SceneEntityCfg("robot").
        object_cfg: The object configuration. Defaults to SceneEntityCfg("object").

    """
    # extract the used quantities (to enable type-hinting)
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    object_pos_w = object.data.root_pos_w[:, :3]
    object_pos_b, _ = subtract_frame_transforms(
        robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], object_pos_w
    )
    des_pos_b = command[:, :3]
    error = torch.norm(des_pos_b - object_pos_b, dim=1)
    # des_pos_w, _ = combine_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b)
    # # distance of the end-effector to the object: (num_envs,)
    # distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)
    # if error.item() < threshold:
    #  #   print(f"Observed Object at goal : {object.data.root_pos_w[:, 2].item()}")
    #     loghelper.logsubtask(LogType.GOAL)
    return error < threshold


def object_reached_midgoal(
    env: ManagerBasedRLEnv,
    command_name: str = "object_pose",
    threshold: float = 0.02,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Termination condition for the object reaching the midgoal position.

    Args:
        env: The environment.
        command_name: The name of the command that is used to control the object.
        threshold: The threshold for the object to reach the goal position. Defaults to 0.02.
        robot_cfg: The robot configuration. Defaults to SceneEntityCfg("robot").
        object_cfg: The object configuration. Defaults to SceneEntityCfg("object").

    """
    # extract the used quantities (to enable type-hinting)
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    object_pos_w = object.data.root_pos_w[:, :3]
    object_pos_b, _ = subtract_frame_transforms(
        robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], object_pos_w
    )
    des_pos_b = command[:, :3]
    error = torch.norm(des_pos_b - object_pos_b, dim=1)
    # des_pos_w, _ = combine_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b)
    # # distance of the end-effector to the object: (num_envs,)
    # distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)
    # if error.item() < threshold:
    #  #   print(f"Observed Object at goal : {object.data.root_pos_w[:, 2].item()}")
    #     loghelper.logsubtask(LogType.MIDGOAL)
    return error < threshold