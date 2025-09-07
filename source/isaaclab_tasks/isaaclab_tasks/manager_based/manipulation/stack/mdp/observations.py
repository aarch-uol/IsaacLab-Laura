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
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    object_2_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
    object_3_cfg: SceneEntityCfg = None,
) -> torch.Tensor:
    """The position of objects in the world frame, supporting 2 or 3 objects."""
    
    object_1: RigidObject = env.scene[object_cfg.name]
    object_2: RigidObject = env.scene[object_2_cfg.name]
    
    # Initialize the list of positions with the first two objects.
    positions_list = [object_1.data.root_pos_w, object_2.data.root_pos_w]
    
    # Conditionally add the third object's position if it exists.
    if object_3_cfg is not None:
        object_3: RigidObject = env.scene[object_3_cfg.name]
        positions_list.append(object_3.data.root_pos_w)
    
    return torch.cat(positions_list, dim=1)

#comment out 3 for 1 task 
def object_orientations_in_world_frame(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    object_2_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
    object_3_cfg: SceneEntityCfg = SceneEntityCfg("object3"),
):
    """The orientation of objects in the world frame, supporting 2 or 3 objects."""
    
    object_1: RigidObject = env.scene[object_cfg.name]
    object_2: RigidObject = env.scene[object_2_cfg.name]
    
    # Initialize the list of orientations with the first two objects
    orientations_list = [object_1.data.root_quat_w, object_2.data.root_quat_w]
    
    # Conditionally add the third object's orientation
    if object_3_cfg is not None:
        object_3: RigidObject = env.scene[object_3_cfg.name]
        orientations_list.append(object_3.data.root_quat_w)
    
    return torch.cat(orientations_list, dim=1)

def object_obs(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    object_2_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
    object_3_cfg: SceneEntityCfg = None,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
):
    """
    Object observations (in world frame) for 2 or 3 objects.
    """
    object_1: RigidObject = env.scene[object_cfg.name]
    object_2: RigidObject = env.scene[object_2_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    object_1_pos_w = object_1.data.root_pos_w
    object_1_quat_w = object_1.data.root_quat_w

    object_2_pos_w = object_2.data.root_pos_w
    object_2_quat_w = object_2.data.root_quat_w

    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]
    
    # Base observations for object1 and object2
    observations = [
        object_1_pos_w - env.scene.env_origins,
        object_1_quat_w,
        object_2_pos_w - env.scene.env_origins,
        object_2_quat_w,
        object_1_pos_w - ee_pos_w, # gripper to object_1
        object_2_pos_w - ee_pos_w, # gripper to object_2
        object_1_pos_w - object_2_pos_w, # object_1 to object_2
    ]
    
    # Conditionally add observations for the third object
    if object_3_cfg is not None:
        object_3: RigidObject = env.scene[object_3_cfg.name]
        object_3_pos_w = object_3.data.root_pos_w
        object_3_quat_w = object_3.data.root_quat_w
        
        observations.extend([
            object_3_pos_w - env.scene.env_origins,
            object_3_quat_w,
            object_3_pos_w - ee_pos_w, # gripper to object_3
            object_1_pos_w - object_3_pos_w, # object_1 to object_3
            object_2_pos_w - object_3_pos_w, # object_2 to object_3
        ])
    
    return torch.cat(observations, dim=1)

def object_obs2(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    object_2_cfg: SceneEntityCfg = SceneEntityCfg("object2"),
    object_3_cfg: SceneEntityCfg = SceneEntityCfg("object3"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
):
    """
    Object observations (in world frame):
        object_1 pos,
        object_1 quat,
        object_2 pos,
        object_2 quat,
        object_3 pos,
        object_3 quat,
        gripper to object_1,
        gripper to object_2,
        gripper to object_3,
        object_1 to object_2,
        object_1 to object_3,
        object_2 to object_3,
    """
    object_1: RigidObject = env.scene[object_cfg.name]
    object_2: RigidObject = env.scene[object_2_cfg.name]
    object_3: RigidObject = env.scene[object_3_cfg.name]
    ee_frame: FrameTransformer = env.scene[ee_frame_cfg.name]

    object_1_pos_w = object_1.data.root_pos_w
    object_1_quat_w = object_1.data.root_quat_w

    object_2_pos_w = object_2.data.root_pos_w
    object_2_quat_w = object_2.data.root_quat_w

    object_3_pos_w = object_3.data.root_pos_w
    object_3_quat_w = object_3.data.root_quat_w

    ee_pos_w = ee_frame.data.target_pos_w[:, 0, :]
    gripper_to_object_1 = object_1_pos_w - ee_pos_w
    gripper_to_object_2 = object_2_pos_w - ee_pos_w
    gripper_to_object_3 = object_3_pos_w - ee_pos_w

    object_1_to_2 = object_1_pos_w - object_2_pos_w
    object_1_to_3 = object_1_pos_w - object_3_pos_w
    object_2_to_3 = object_2_pos_w - object_3_pos_w

    return torch.cat(
        (
            object_1_pos_w - env.scene.env_origins,
            object_1_quat_w,
            object_2_pos_w - env.scene.env_origins,
            object_2_quat_w,
            object_3_pos_w - env.scene.env_origins,
            object_3_quat_w,
            gripper_to_object_1,
            gripper_to_object_2,
            gripper_to_object_3,
            object_1_to_2,
            object_1_to_3,
            object_2_to_3,
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
    object_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    threshold: float = 0.05,
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
   # if object_ee_distance < threshold:
     #   print("SUBTASK 1 : reach_object")
    return object_ee_distance < threshold


def object_grasped(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
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
    #if grasped[0]:
       # print("SUBTASK : grasped")
    # if grasped[0]:
    #     loghelper.logsubtask(LogType.GRASP)

    return grasped

def is_object_lifted(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg,
    threshold : float = 0.05,
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    #return true when object z coord above a threshold value 
    object = env.scene[object_cfg.name]
    # if object.data.root_pos_w[:, 2].item() > threshold : 
    #     loghelper.logsubtask(LogType.LIFT)
    #if object.data.root_pos_w[:, 2] > threshold:
    #    print("SUBTASK : lifted")
    return object.data.root_pos_w[:, 2] > threshold

### Add a waypoint here

def reach_object2(
    env: ManagerBasedRLEnv,
    object_cfg: SceneEntityCfg,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
    threshold: float = 0.2,
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
    #if object2_ee_distance < (threshold + 0.05):
    #    print("SUBTASK : reach_object2")
    return object2_ee_distance < (threshold + 0.05)

def pouring_solution(
    env: ManagerBasedEnv,
    angle_threshold: int = 45,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
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
    #if angles_deg < angle:
     #   print("SUBTASK : pour_object")
    return angles_deg < angle

def reorient_object(
    env: ManagerBasedEnv,
    angle_threshold: int = 175,
    ee_frame_cfg: SceneEntityCfg = SceneEntityCfg("ee_frame"),
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
    #if angles_deg > angle:
    #    print("SUBTASK : reorient_object")
    return angles_deg > angle

def object_position_in_robot_root_frame(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    
) -> torch.Tensor:
    """The position of the object in the robot's root frame."""
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
    object_pos_w = object.data.root_pos_w[:, :3]
    object_pos_b, _ = subtract_frame_transforms(
        robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], object_pos_w
    )
   
    return object_pos_b

def object_stacked(
    env: ManagerBasedRLEnv,
    # Upper and lower object cfg defined in stack_env_cfg when stacked in subtask
    upper_object_cfg: SceneEntityCfg ,#= SceneEntityCfg("object1"),
    lower_object_cfg: SceneEntityCfg ,#=SceneEntityCfg("object3"),
    command_name: str = "object_pose",
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    xy_threshold: float = 0.1,
    height_threshold: float = 0.006,
    height_diff: float = 0.05,
    threshold: float = 0.05,
    gripper_open_val: torch.tensor = torch.tensor([0.04]),
    atol=0.0001,
    rtol=0.0001,
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Check if an object is stacked by the specified robot."""

    robot: Articulation = env.scene[robot_cfg.name]
    upper_object: RigidObject = env.scene[upper_object_cfg.name]
    lower_object: RigidObject = env.scene[lower_object_cfg.name]
    command = env.command_manager.get_command(command_name)

    # pos_diff = upper_object.data.root_pos_w - lower_object.data.root_pos_w
    # height_dist = torch.linalg.vector_norm(pos_diff[:, 2:], dim=1)
    # xy_dist = torch.linalg.vector_norm(pos_diff[:, :2], dim=1)

    # stacked = torch.logical_and(xy_dist < xy_threshold, (height_dist - height_diff) < height_threshold)

    # # Checking right and left gripper positions
    # stacked = torch.logical_and(
    #     torch.isclose(robot.data.joint_pos[:, -1], gripper_open_val.to(env.device), atol=1e-4, rtol=1e-4), stacked
    # )
    # stacked = torch.logical_and(
    #     torch.isclose(robot.data.joint_pos[:, -2], gripper_open_val.to(env.device), atol=1e-4, rtol=1e-4), stacked
    # )
    # if stacked[0]:
    #     loghelper.logsubtask(LogType.STACK)
        # Position difference between cube and hot plate 
    pos_diff = lower_object.data.root_pos_w - upper_object.data.root_pos_w

    # # Compute cube position difference in x-y plane
    xy_dist = torch.norm(pos_diff[:, :2], dim=1)

    height_diff_actual = pos_diff[:, 2]

    # Is true when reaches threshold
    xy_check = xy_dist < xy_threshold

    # Check that the object is stacked
    # overall_height = height_diff_actual - height_diff
    overall_height = height_diff_actual + height_diff
    # print(overall_height)
    # height_check = torch.abs(overall_height) < height_threshold
    height_check = torch.where(overall_height < 0, overall_height, torch.abs(overall_height))

    # Combine [True] for all dimensions
    stacked = torch.logical_and(xy_check, height_check)
    #if stacked == True:
       # print("SUBTASK : object_stacked")

    return stacked
    
def objects_stacked(
    env: ManagerBasedRLEnv,
    # Upper and lower object cfg defined in stack_env_cfg when stacked in subtask
    upper_object_cfg: SceneEntityCfg,
    lower_object_cfg: SceneEntityCfg,
    command_name: str = "object_pose",
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    xy_threshold: float = 0.1,
    height_threshold: float = 0.006,
    height_diff: float = 0.05,
    threshold: float = 0.05,
    gripper_open_val: torch.tensor = torch.tensor([0.04]),
    atol=0.0001,
    rtol=0.0001,
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Check if an object is stacked by the specified robot."""

    robot: Articulation = env.scene[robot_cfg.name]
    upper_object: RigidObject = env.scene[upper_object_cfg.name]
    lower_object: RigidObject = env.scene[lower_object_cfg.name]
    command = env.command_manager.get_command(command_name)

    # pos_diff = upper_object.data.root_pos_w - lower_object.data.root_pos_w
    # height_dist = torch.linalg.vector_norm(pos_diff[:, 2:], dim=1)
    # xy_dist = torch.linalg.vector_norm(pos_diff[:, :2], dim=1)

    # stacked = torch.logical_and(xy_dist < xy_threshold, (height_dist - height_diff) < height_threshold)

    # # Checking right and left gripper positions
    # stacked = torch.logical_and(
    #     torch.isclose(robot.data.joint_pos[:, -1], gripper_open_val.to(env.device), atol=1e-4, rtol=1e-4), stacked
    # )
    # stacked = torch.logical_and(
    #     torch.isclose(robot.data.joint_pos[:, -2], gripper_open_val.to(env.device), atol=1e-4, rtol=1e-4), stacked
    # )
    # if stacked[0]:
    #     loghelper.logsubtask(LogType.STACK)
        # Position difference between cube and hot plate 
    pos_diff = lower_object.data.root_pos_w - upper_object.data.root_pos_w

    # # Compute cube position difference in x-y plane
    xy_dist = torch.norm(pos_diff[:, :2], dim=1)

    height_diff_actual = pos_diff[:, 2]

    # Is true when reaches threshold
    xy_check = xy_dist < xy_threshold

    # Check that the object is stacked
    # overall_height = height_diff_actual - height_diff
    overall_height = height_diff_actual + height_diff
    # print(overall_height)
    # height_check = torch.abs(overall_height) < height_threshold
    height_check = torch.where(overall_height < 0, overall_height, torch.abs(overall_height))

    # Combine [True] for all dimensions
    stacked = torch.logical_and(xy_check, height_check)
    #if stacked == True:
       # print("SUBTASK : object_stacked")

    return stacked

def object_near_goal(
    env: ManagerBasedRLEnv,
    # command_name: str = "object_pose",
    object_cfg: SceneEntityCfg,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    threshold: float = 0.02,
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
    # command = env.command_manager.get_command(command_name)
    # compute the desired position in the world frame
    # object_pos_w = object.data.root_pos_w[:, :3]
    # object_pos_b, _ = subtract_frame_transforms(
    #     robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], object_pos_w
    # )
    # des_pos_b = command[:, :3]
    # error = torch.norm(des_pos_b - object_pos_b, dim=1)
    # des_pos_w, _ = combine_frame_transforms(robot.data.root_state_w[:, :3], robot.data.root_state_w[:, 3:7], des_pos_b)
    # # distance of the end-effector to the object: (num_envs,)
    # distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)
    # if error.item() < threshold:
    #  #   print(f"Observed Object at goal : {object.data.root_pos_w[:, 2].item()}")
    #     loghelper.logsubtask(LogType.GOAL)
    # return error < threshold
        # Desired position in robot base frame
    des_pos_b = torch.tensor([0.6, 0.0, 0.0203], device=robot.data.root_pos_w.device)
    # Transform desired pos to world frame
    des_pos_w, _ = combine_frame_transforms(
        robot.data.root_state_w[:, :3],        # root position
        robot.data.root_state_w[:, 3:7],       # root rotation (quat)
        des_pos_b
    )
    # Compute distance to object
    distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)  # shape: (num_envs,)
    # Log if any environment meets condition
    mask = distance < threshold
    # if torch.any(mask):
    #     loghelper.logsubtask(LogType.FINISH)
    #if distance < threshold:
    #    print("Obs func: object_near_goal")
    return mask 


def object_reached_midgoal(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    command_name: str = "object_pose",
    threshold: float = 0.02,
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
    #if error < threshold:
     #   print("Obs function: object_reached_midgoal")
    return error < threshold


    import omni.log

def get_joint_pos(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
) -> torch.Tensor:
    """Check if an object is grasped by the specified robot."""

    robot: Articulation = env.scene[robot_cfg.name]
   # print(f"[OBS] robot_pose :  {robot.data.joint_pos}")
    return robot.data.joint_pos
# 
# class ObjectsStacked:
#     def __init__(
#         self,
#         lower_object_cfg: SceneEntityCfg,
#         robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
#         object_1_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
#         command_name: str = "object_pose",
#         xy_threshold: float = 0.1,
#         height_threshold: float = 0.006,
#         height_diff: float = 0.05,
#         gripper_open_val: torch.Tensor = torch.tensor([0.04]),
#         atol: float = 1e-4,
#         rtol: float = 1e-4,
#         success_hold_time: int = 500,
#         success_hold_steps: int = 500, 
#     ):
#         """Termination condition for detecting stacked objects with a hold time."""
#         self.lower_object_cfg = lower_object_cfg
#         self.robot_cfg = robot_cfg
#         self.object_1_cfg = object_1_cfg
#         self.command_name = command_name
#         self.xy_threshold = xy_threshold
#         self.height_threshold = height_threshold
#         self.height_diff = height_diff
#         self.gripper_open_val = gripper_open_val
#         self.atol = atol
#         self.rtol = rtol
#         self.success_hold_steps = success_hold_steps

#         # internal state
#         self._consecutive_steps = 0

#     def __call__(self, env: ManagerBasedRLEnv):
#         """Check if object_1 is stacked on lower_object for a continuous number of steps."""
#         robot: Articulation = env.scene[self.robot_cfg.name]
#         object_1: RigidObject = env.scene[self.object_1_cfg.name]
#         object_2: RigidObject = env.scene[self.lower_object_cfg.name]

#         # position difference between objects
#         pos_diff = object_2.data.root_pos_w - object_1.data.root_pos_w
#         xy_dist = torch.norm(pos_diff[:, :2], dim=1)
#         height_diff_actual = pos_diff[:, 2]

#         # spatial checks
#         xy_check = xy_dist < self.xy_threshold
#         overall_height = height_diff_actual + self.height_diff
#         height_check = torch.where(
#             overall_height < 0, overall_height, torch.abs(overall_height)
#         )
#         stacked = torch.logical_and(xy_check, height_check)

#         # check gripper is open
#         stacked = torch.logical_and(
#             torch.isclose(
#                 robot.data.joint_pos[:, -1],
#                 self.gripper_open_val.to(env.device),
#                 atol=self.atol,
#                 rtol=self.rtol,
#             ),
#             stacked,
#         )
#         stacked = torch.logical_and(
#             torch.isclose(
#                 robot.data.joint_pos[:, -2],
#                 self.gripper_open_val.to(env.device),
#                 atol=self.atol,
#                 rtol=self.rtol,
#             ),
#             stacked,
#         )
#         omni.log.info(f"[ObjectsStacked] stacked tensor: {stacked}, consecutive steps: {self._consecutive_steps}")
#         # step-count-based hold logic
#         if stacked.all():
#             print("stacked....")
#             self._consecutive_steps += 1
#             omni.log.info(f"in place for : {self._consecutive_steps}")
#             print(f"in place for : {self._consecutive_steps}")
#             if self._consecutive_steps >= self.success_hold_steps:
#                 print(f"✅ Termination: objects_stacked held for {self.success_hold_steps} steps")
#                 self._consecutive_steps = 0
#                 return torch.tensor([True], device=env.device)
#         else:
#             self._consecutive_steps = 0  # reset if not stacked

#         return torch.tensor([False], device=env.device)