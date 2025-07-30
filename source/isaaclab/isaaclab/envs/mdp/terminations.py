<<<<<<< HEAD
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e (Fresh start, no history)
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Common functions that can be used to activate certain terminations.

The functions can be passed to the :class:`isaaclab.managers.TerminationTermCfg` object to enable
the termination introduced by the function.
"""

from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor
<<<<<<< HEAD

=======
from isaaclab.utils.logging_helper import LoggingHelper, ErrorType, LogType
>>>>>>> abfba5273e (Fresh start, no history)
if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers.command_manager import CommandTerm

<<<<<<< HEAD
=======

>>>>>>> abfba5273e (Fresh start, no history)
"""
MDP terminations.
"""


<<<<<<< HEAD
def time_out(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate the episode when the episode length exceeds the maximum episode length."""
    return env.episode_length_buf >= env.max_episode_length


def command_resample(env: ManagerBasedRLEnv, command_name: str, num_resamples: int = 1) -> torch.Tensor:
=======
def time_out(env: ManagerBasedRLEnv, loghelper : LoggingHelper = LoggingHelper()) -> torch.Tensor:
    """Terminate the episode when the episode length exceeds the maximum episode length."""
    # if torch.max(env.episode_length_buf) >= env.max_episode_length:
    #     loghelper.logerror(ErrorType.TIMEOUT)        
    return env.episode_length_buf >= env.max_episode_length


def command_resample(env: ManagerBasedRLEnv, command_name: str, num_resamples: int = 1, loghelper : LoggingHelper = LoggingHelper()) -> torch.Tensor:
>>>>>>> abfba5273e (Fresh start, no history)
    """Terminate the episode based on the total number of times commands have been re-sampled.

    This makes the maximum episode length fluid in nature as it depends on how the commands are
    sampled. It is useful in situations where delayed rewards are used :cite:`rudin2022advanced`.
    """
    command: CommandTerm = env.command_manager.get_term(command_name)
<<<<<<< HEAD
=======
    # if torch.logical_and((command.time_left <= env.step_dt), (command.command_counter == num_resamples)).item():
    #     loghelper.logerror(ErrorType.RESAMPLE)
>>>>>>> abfba5273e (Fresh start, no history)
    return torch.logical_and((command.time_left <= env.step_dt), (command.command_counter == num_resamples))


"""
Root terminations.
"""


<<<<<<< HEAD
def bad_orientation(
    env: ManagerBasedRLEnv, limit_angle: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Terminate when the asset's orientation is too far from the desired orientation limits.

    This is computed by checking the angle between the projected gravity vector and the z-axis.
    """
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
=======
# def bad_orientation(
#     env: ManagerBasedRLEnv, limit_angle: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"), 
#     loghelper : LoggingHelper = LoggingHelper()
# ) -> torch.Tensor:
#     """Terminate when the asset's orientation is too far from the desired orientation limits.

#     This is computed by checking the angle between the projected gravity vector and the z-axis.
#     """
#     # extract the used quantities (to enable type-hinting)
#     asset: RigidObject = env.scene[asset_cfg.name]
#     # if (torch.acos(-asset.data.projected_gravity_b[:, 2]).abs()).item() > limit_angle:
#     #     loghelper.logerror(ErrorType.ORIENTATION)
#     return torch.acos(-asset.data.projected_gravity_b[:, 2]).abs() > limit_angle


def bad_orientation(
    env: ManagerBasedRLEnv,
    limit_angle: float,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    loghelper: LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """
    Terminate when the asset's orientation deviates too far from its original upright pose.
    Deviation is computed from the initial projected gravity vector.
    """
    asset: RigidObject = env.scene[asset_cfg.name]

    # Use persistent state on the function to store upright baseline
    if not hasattr(bad_orientation, "_baseline_upright"):
        bad_orientation._baseline_upright = torch.acos(-asset.data.projected_gravity_b[:, 2]).abs().detach()
    current_angle = torch.acos(-asset.data.projected_gravity_b[:, 2])
    deviation = (current_angle - bad_orientation._baseline_upright).abs()
    if torch.any(deviation > limit_angle):
        print(f"[DEBUG] Upright deviation: {deviation.item():.4f} rad (limit: {limit_angle})")
    return deviation > limit_angle
    if (torch.acos(-asset.data.projected_gravity_b[:, 2]).abs()).item() > limit_angle:
    #     loghelper.logerror(ErrorType.ORIENTATION)
        print("bad orientation")
>>>>>>> abfba5273e (Fresh start, no history)
    return torch.acos(-asset.data.projected_gravity_b[:, 2]).abs() > limit_angle


def root_height_below_minimum(
<<<<<<< HEAD
    env: ManagerBasedRLEnv, minimum_height: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
) -> torch.Tensor:
    """Terminate when the asset's root height is below the minimum height.

=======
    env: ManagerBasedRLEnv, minimum_height: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    loghelper : LoggingHelper = LoggingHelper()
) -> torch.Tensor:
    """Terminate when the asset's root height is below the minimum height.
>>>>>>> abfba5273e (Fresh start, no history)
    Note:
        This is currently only supported for flat terrains, i.e. the minimum height is in the world frame.
    """
    # extract the used quantities (to enable type-hinting)
    asset: RigidObject = env.scene[asset_cfg.name]
<<<<<<< HEAD
=======
    if torch.any(asset.data.root_pos_w[:, 2] < minimum_height):
        # loghelper.logerror(ErrorType.DROP)
        print(f"[DEBUG] Asset {asset_cfg.name} dropped below minimum height: {asset.data.root_pos_w[:, 2].item()} < {minimum_height}")
>>>>>>> abfba5273e (Fresh start, no history)
    return asset.data.root_pos_w[:, 2] < minimum_height


"""
Joint terminations.
"""


<<<<<<< HEAD
def joint_pos_out_of_limit(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
=======
def joint_pos_out_of_limit(
        env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
        loghelper : LoggingHelper = LoggingHelper()) -> torch.Tensor:
>>>>>>> abfba5273e (Fresh start, no history)
    """Terminate when the asset's joint positions are outside of the soft joint limits."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    if asset_cfg.joint_ids is None:
        asset_cfg.joint_ids = slice(None)

    limits = asset.data.soft_joint_pos_limits[:, asset_cfg.joint_ids]
    out_of_upper_limits = torch.any(asset.data.joint_pos[:, asset_cfg.joint_ids] > limits[..., 1], dim=1)
    out_of_lower_limits = torch.any(asset.data.joint_pos[:, asset_cfg.joint_ids] < limits[..., 0], dim=1)
    return torch.logical_or(out_of_upper_limits, out_of_lower_limits)


def joint_pos_out_of_manual_limit(
<<<<<<< HEAD
    env: ManagerBasedRLEnv, bounds: tuple[float, float], asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
=======
    env: ManagerBasedRLEnv, bounds: tuple[float, float], asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    loghelper : LoggingHelper = LoggingHelper()
>>>>>>> abfba5273e (Fresh start, no history)
) -> torch.Tensor:
    """Terminate when the asset's joint positions are outside of the configured bounds.

    Note:
        This function is similar to :func:`joint_pos_out_of_limit` but allows the user to specify the bounds manually.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    if asset_cfg.joint_ids is None:
        asset_cfg.joint_ids = slice(None)
    # compute any violations
    out_of_upper_limits = torch.any(asset.data.joint_pos[:, asset_cfg.joint_ids] > bounds[1], dim=1)
    out_of_lower_limits = torch.any(asset.data.joint_pos[:, asset_cfg.joint_ids] < bounds[0], dim=1)
<<<<<<< HEAD
    return torch.logical_or(out_of_upper_limits, out_of_lower_limits)


def joint_vel_out_of_limit(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")) -> torch.Tensor:
=======
    if torch.logical_or(out_of_upper_limits, out_of_lower_limits).item():
    #     loghelper.logerror(ErrorType.JOINT_VIO)
        print("joint pos violation")
    
    return torch.logical_or(out_of_upper_limits, out_of_lower_limits)


def joint_vel_out_of_limit(
        env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
        loghelper : LoggingHelper = LoggingHelper()) -> torch.Tensor:
>>>>>>> abfba5273e (Fresh start, no history)
    """Terminate when the asset's joint velocities are outside of the soft joint limits."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # compute any violations
    limits = asset.data.soft_joint_vel_limits
<<<<<<< HEAD
=======
    if torch.any(torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids]) > limits[:, asset_cfg.joint_ids], dim=1).item():
    #     loghelper.logerror(ErrorType.VEL_VIO)
        print("Joint violation")
>>>>>>> abfba5273e (Fresh start, no history)
    return torch.any(torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids]) > limits[:, asset_cfg.joint_ids], dim=1)


def joint_vel_out_of_manual_limit(
<<<<<<< HEAD
    env: ManagerBasedRLEnv, max_velocity: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
=======
    env: ManagerBasedRLEnv, max_velocity: float, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    loghelper : LoggingHelper = LoggingHelper()
>>>>>>> abfba5273e (Fresh start, no history)
) -> torch.Tensor:
    """Terminate when the asset's joint velocities are outside the provided limits."""
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
    # compute any violations
<<<<<<< HEAD
=======
    if torch.any(torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids]) > max_velocity, dim=1).item():
        print("joint velocity violation")
    #     loghelper.logerror(ErrorType.VEL_VIO)
>>>>>>> abfba5273e (Fresh start, no history)
    return torch.any(torch.abs(asset.data.joint_vel[:, asset_cfg.joint_ids]) > max_velocity, dim=1)


def joint_effort_out_of_limit(
<<<<<<< HEAD
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot")
=======
    env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    loghelper : LoggingHelper = LoggingHelper()
>>>>>>> abfba5273e (Fresh start, no history)
) -> torch.Tensor:
    """Terminate when effort applied on the asset's joints are outside of the soft joint limits.

    In the actuators, the applied torque are the efforts applied on the joints. These are computed by clipping
    the computed torques to the joint limits. Hence, we check if the computed torques are equal to the applied
    torques.
    """
    # extract the used quantities (to enable type-hinting)
    asset: Articulation = env.scene[asset_cfg.name]
<<<<<<< HEAD
    # check if any joint effort is out of limit
    out_of_limits = torch.isclose(
        asset.data.computed_torque[:, asset_cfg.joint_ids], asset.data.applied_torque[:, asset_cfg.joint_ids]
    )
=======
    applied_torque = asset.data.applied_torque[:, asset_cfg.joint_ids]
    # check if any joint effort is out of limit
    out_of_limits = torch.isclose(
        applied_torque, asset.data.joint_effort_limits
    )
    if torch.any(torch.any(out_of_limits, dim=1)):
        print(f"joint effort violation: limit {asset.data.joint_effort_limits}:  \n applied torque : {asset.data.applied_torque[:, asset_cfg.joint_ids]} ")
    #     loghelper.logerror(ErrorType.EFF_VIO)
>>>>>>> abfba5273e (Fresh start, no history)
    return torch.any(out_of_limits, dim=1)


"""
Contact sensor.
"""


<<<<<<< HEAD
def illegal_contact(env: ManagerBasedRLEnv, threshold: float, sensor_cfg: SceneEntityCfg) -> torch.Tensor:
=======
def illegal_contact(
        env: ManagerBasedRLEnv, threshold: float, sensor_cfg: SceneEntityCfg,
        loghelper : LoggingHelper = LoggingHelper()) -> torch.Tensor:
>>>>>>> abfba5273e (Fresh start, no history)
    """Terminate when the contact force on the sensor exceeds the force threshold."""
    # extract the used quantities (to enable type-hinting)
    contact_sensor: ContactSensor = env.scene.sensors[sensor_cfg.name]
    net_contact_forces = contact_sensor.data.net_forces_w_history
    # check if any contact force exceeds the threshold
<<<<<<< HEAD
=======
    if torch.any(
        torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold, dim=1
    ).item():
        loghelper.logerror(ErrorType.CONTACT)
        print("Illegal contact")
        
>>>>>>> abfba5273e (Fresh start, no history)
    return torch.any(
        torch.max(torch.norm(net_contact_forces[:, :, sensor_cfg.body_ids], dim=-1), dim=1)[0] > threshold, dim=1
    )
