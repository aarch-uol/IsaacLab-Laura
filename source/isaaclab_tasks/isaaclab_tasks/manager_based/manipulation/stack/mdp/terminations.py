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

import time
import omni.log
# 
class ObjectsStacked:
    def __init__(
        self,
        lower_object_cfg: SceneEntityCfg,
        robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
        object_1_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
        xy_threshold: float = 0.1,
        height_threshold: float = 0.05,
        height_diff: float = 0.05,
        gripper_open_val: torch.Tensor = torch.tensor([0.04]),
        atol: float = 1e-4,
        rtol: float = 1e-4,
        success_hold_steps: int = 500,
    ):
        """Termination condition for detecting stacked objects with per-env hold steps."""
        self.lower_object_cfg = lower_object_cfg
        self.robot_cfg = robot_cfg
        self.object_1_cfg = object_1_cfg
        self.xy_threshold = xy_threshold
        self.height_threshold = height_threshold
        self.height_diff = height_diff
        self.gripper_open_val = gripper_open_val
        self.atol = atol
        self.rtol = rtol
        self.success_hold_steps = success_hold_steps

        # Will be allocated after we know num_envs
        self._consecutive_steps = None

    def __call__(self, env: ManagerBasedRLEnv) -> torch.Tensor:
        """Return per-env success mask: True if stacked for required hold steps."""
        robot: Articulation = env.scene[self.robot_cfg.name]
        object_1: RigidObject = env.scene[self.object_1_cfg.name]
        object_2: RigidObject = env.scene[self.lower_object_cfg.name]

        num_envs = env.unwrapped.num_envs
        if self._consecutive_steps is None or self._consecutive_steps.shape[0] != num_envs:
            self._consecutive_steps = torch.zeros(num_envs, dtype=torch.long, device=env.device)

        # position difference between objects
        pos_diff = object_2.data.root_pos_w - object_1.data.root_pos_w
        xy_dist = torch.norm(pos_diff[:, :2], dim=1)
       # print("xy dist error : ", xy_dist)
        height_diff_actual = pos_diff[:, 2]

        # spatial checks
        xy_check = xy_dist < self.xy_threshold
       # print("xy error check : ", xy_check)
        overall_height = height_diff_actual + self.height_diff
        height_check = torch.abs(overall_height) < self.height_threshold
        #print("height diff : ", height_check)
        stacked = torch.logical_and(xy_check, height_check)
        #print("Stacked ? : ", stacked)
        # gripper open check
        open_check = torch.logical_and(
            torch.isclose(
                robot.data.joint_pos[:, -1],
                self.gripper_open_val.to(env.device),
                atol=self.atol,
                rtol=self.rtol,
            ),
            torch.isclose(
                robot.data.joint_pos[:, -2],
                self.gripper_open_val.to(env.device),
                atol=self.atol,
                rtol=self.rtol,
            ),
        )

        stacked = torch.logical_and(stacked, open_check)
        #print("Stacked ? : ", stacked)
        # update per-env counters
        self._consecutive_steps = torch.where(
            stacked,
            self._consecutive_steps + 1,
            torch.zeros_like(self._consecutive_steps),
        )

        # mark success if hold threshold reached
        success = self._consecutive_steps >= self.success_hold_steps

        # reset those envs that just succeeded (prevents sticky True)
        self._consecutive_steps = torch.where(
            success,
            torch.zeros_like(self._consecutive_steps),
            self._consecutive_steps,
        )
        #print("Success terms : ", success)
        return success



def object_reached_goal(
    env: ManagerBasedRLEnv,
    threshold: float = 0.05,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    object_cfg: SceneEntityCfg = SceneEntityCfg("object1"),
    # loghelper : LoggingHelper = LoggingHelper()
    gripper_open_val: torch.tensor = torch.tensor([0.04]),
    atol=0.0001,
    rtol=0.0001,
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
    des_pos_b = torch.tensor([0.6, -0.2, 0.0203], device=robot.data.root_pos_w.device)
    # Transform desired pos to world frame
    des_pos_w, _ = combine_frame_transforms(
        robot.data.root_state_w[:, :3],        # root position
        robot.data.root_state_w[:, 3:7],       # root rotation (quat)
        des_pos_b
    )
    # Compute distance to object
    distance = torch.norm(des_pos_w - object.data.root_pos_w[:, :3], dim=1)  # shape: (num_envs,)
    # Log if any environment meets condition
    # if torch.any(mask):
    #     loghelper.logsubtask(LogType.FINISH)
    grasp = torch.abs(distance < threshold)
    grasp = torch.logical_and(
        torch.isclose(robot.data.joint_pos[:, -1], gripper_open_val.to(env.device), atol=atol, rtol=rtol), grasp
    )
    grasp = torch.logical_and(
        torch.isclose(robot.data.joint_pos[:, -2], gripper_open_val.to(env.device), atol=atol, rtol=rtol), grasp
    )
    if distance < threshold:
        print("Term function: object_reached_goal")
    return grasp
    