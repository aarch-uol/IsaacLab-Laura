# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations for the lift task.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
# from isaaclab.utils.logging_helper import LoggingHelper, ErrorType, LogType
from isaaclab.utils.math import subtract_frame_transforms, combine_frame_transforms
if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def objects_stacked(
    env: ManagerBasedRLEnv,
    object_2_cfg: SceneEntityCfg, 
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_1_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    command_name: str = "object_pose",
    xy_threshold: float = 0.1,
    height_threshold: float = 0.006,
    # height_diff: float = 0.0468,
    height_diff: float = 0.05,
    threshold: float = 0.02,
    gripper_open_val: torch.tensor = torch.tensor([0.04]),
    atol=0.0001,
    rtol=0.0001,
    # loghelper : LoggingHelper = LoggingHelper(),
):
    robot: Articulation = env.scene[robot_cfg.name]
    object_1: RigidObject = env.scene[object_1_cfg.name]
    object_2: RigidObject = env.scene[object_2_cfg.name]
    command = env.command_manager.get_command(command_name)

    # Position difference between cube and hot plate 
    pos_diff = object_2.data.root_pos_w - object_1.data.root_pos_w

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
    # Check gripper positions (right and left) are open
    stacked = torch.logical_and(
        torch.isclose(robot.data.joint_pos[:, -1], gripper_open_val.to(env.device), atol=atol, rtol=rtol), stacked
    )
    stacked = torch.logical_and(
        torch.isclose(robot.data.joint_pos[:, -2], gripper_open_val.to(env.device), atol=atol, rtol=rtol), stacked
    )
    if stacked == True:
        print("Termination function: objects_stacked")

    return stacked


def object_reached_goal(
    env: ManagerBasedRLEnv,
    threshold: float = 0.02,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    # loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Termination condition for the object reaching the goal position.

    Args:
        env: The environment.
        threshold: The threshold for the object to reach the goal position. Defaults to 0.02.
        robot_cfg: The robot configuration. Defaults to SceneEntityCfg("robot").
        object_cfg: The object configuration. Defaults to SceneEntityCfg("object").

    """
    #### Currently won't show where this is
    robot: RigidObject = env.scene[robot_cfg.name]
    object: RigidObject = env.scene[object_cfg.name]
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
    if distance < threshold:
        print("Term function: object_reached_goal")
    return mask  # or return distance < threshold