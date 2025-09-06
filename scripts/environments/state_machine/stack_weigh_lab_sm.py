# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
Script to run an environment with a pick and lift state machine.

The state machine is implemented in the kernel function `infer_state_machine`.
It uses the `warp` library to run the state machine in parallel on the GPU.

.. code-block:: bash

    ../isaaclab.sh -p scripts/environments/state_machine/lift_cube_sm-copy.py --num_envs 1

"""

"""Launch Omniverse Toolkit first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
# parser = argparse.ArgumentParser(description="Pick and lift state machine for lift environments.")
# parser.add_argument(
#     "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
# )
# parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
# # append AppLauncher cli args
# AppLauncher.add_app_launcher_args(parser)
# # parse the arguments
# args_cli = parser.parse_args()

# # launch omniverse app
# app_launcher = AppLauncher(headless=args_cli.headless)
# simulation_app = app_launcher.app

"""Rest everything else."""

import gymnasium as gym
import torch
from collections.abc import Sequence

import warp as wp
import numpy as np
from isaaclab.assets.rigid_object.rigid_object_data import RigidObjectData

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.manager_based.manipulation.lift.lift_env_cfg import LiftEnvCfg
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

# initialize warp
wp.init()


class GripperState:
    """States for the gripper."""

    OPEN = wp.constant(1.0)
    CLOSE = wp.constant(-1.0)


class PickSmState:
    """States for the pick state machine."""

    REST = wp.constant(0)
    APPROACH_ABOVE_OBJECT = wp.constant(1)
    APPROACH_OBJECT = wp.constant(2)
    GRASP_OBJECT = wp.constant(3)
    LIFT_OBJECT = wp.constant(4)
    APPROACH_ABOVE_OBJECT2 = wp.constant(5)
    APPROACH_OBJECT2 = wp.constant(6)
    UNGRASP_OBJECT = wp.constant(7)
    REGRASP_OBJECT_2 = wp.constant(8)
    APPROACH_OBJECT_2 = wp.constant(9)
    GRASP_OBJECT_2 = wp.constant(10)
    LIFT_OBJECT_2 = wp.constant(11)
    APPROACH_ABOVE_OBJECT3 = wp.constant(12)
    APPROACH_OBJECT3 = wp.constant(13)
    UNGRASP_OBJECT_2 = wp.constant(14)
    POST_REST = wp.constant(15)


class PickSmWaitTime:
    """Additional wait times (in s) for states for before switching."""

    REST = wp.constant(1)
    APPROACH_ABOVE_OBJECT = wp.constant(0.5)
    APPROACH_OBJECT = wp.constant(0.6)
    GRASP_OBJECT = wp.constant(0.3)
    LIFT_OBJECT = wp.constant(0.5)
    APPROACH_ABOVE_OBJECT2 = wp.constant(0.5)
    APPROACH_OBJECT2 = wp.constant(0.6)
    UNGRASP_OBJECT = wp.constant(5)
    REGRASP_OBJECT_2 = wp.constant(0.5)
    APPROACH_OBJECT_2 = wp.constant(0.6)
    GRASP_OBJECT_2 = wp.constant(0.3)
    LIFT_OBJECT_2 = wp.constant(0.5)
    APPROACH_ABOVE_OBJECT3 = wp.constant(0.5)
    APPROACH_OBJECT3 = wp.constant(0.6)
    UNGRASP_OBJECT_2 = wp.constant(0.5)
    POST_REST = wp.constant(10)


@wp.func
def distance_below_threshold(current_pos: wp.vec3, desired_pos: wp.vec3, threshold: float) -> bool:
    return wp.length(current_pos - desired_pos) < threshold


@wp.kernel
def infer_state_machine(
    dt: wp.array(dtype=float),
    sm_state: wp.array(dtype=int),
    sm_wait_time: wp.array(dtype=float),
    ee_pose: wp.array(dtype=wp.transform),
    object_pose: wp.array(dtype=wp.transform),
    des_object_pose: wp.array(dtype=wp.transform),
    final_object_pose: wp.array(dtype=wp.transform),
    object_task2_pose: wp.array(dtype=wp.transform),
    des_ee_pose: wp.array(dtype=wp.transform),
    gripper_state: wp.array(dtype=float),
    offset: wp.array(dtype=wp.transform),
    position_threshold: float,
    debug_des_pose: wp.array(dtype=wp.transform),   # NEW
    debug_cur_pose: wp.array(dtype=wp.transform),   # NEW
):
    # retrieve thread id
    tid = wp.tid()
    # retrieve state machine state
    state = sm_state[tid]
    # decide next state
    if state == PickSmState.REST:
        #print("[SM_INFO] : REST")
        des_ee_pose[tid] = ee_pose[tid]
        gripper_state[tid] = GripperState.OPEN
        # wait for a while
        if sm_wait_time[tid] >= PickSmWaitTime.REST:
            #print("[SM_INFO] : Moving from REST to APPROACH_ABOVE_OBJECT")
            # move to next state and reset wait time
            sm_state[tid] = PickSmState.APPROACH_ABOVE_OBJECT
            sm_wait_time[tid] = 0.0
    elif state == PickSmState.APPROACH_ABOVE_OBJECT:
        #print("[SM_INFO] : APPROACH_ABOVE_OBJECT")
        des_ee_pose[tid] = wp.transform_multiply(offset[tid], object_pose[tid])
        gripper_state[tid] = GripperState.OPEN
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            0.02,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickSmWaitTime.APPROACH_ABOVE_OBJECT:
                # move to next state and reset wait time
               # print("[SM_INFO] : Moving from APPR_ABOVE to APPROACH_OBJECT")
                sm_state[tid] = PickSmState.APPROACH_OBJECT
                sm_wait_time[tid] = 0.0
        #else: print("[SM_INFO] : not in APPR_ABOVE position ")
    elif state == PickSmState.APPROACH_OBJECT:
        #print("[SM_INFO] : APPROACH_OBJECT")
        pose_pos = wp.transform_get_translation(object_pose[tid])
        pose_rot = wp.transform_get_rotation(object_pose[tid])
        # Apply offset in x-direction (5 cm = 0.05 m) and z direction
        offset_pos = wp.vec3(pose_pos.x , pose_pos.y, pose_pos.z+0.05)
        des_ee_pose[tid] = wp.transform(offset_pos, pose_rot)
        gripper_state[tid] = GripperState.OPEN
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold,
        ):
            if sm_wait_time[tid] >= PickSmWaitTime.APPROACH_OBJECT:
                # move to next state and reset wait time
                #print("[SM_INFO] : Moving from APPR_OBJ to GRASP_OBJECT")
                sm_state[tid] = PickSmState.GRASP_OBJECT
                sm_wait_time[tid] = 0.0
       # else: print("[SM_INFO] : not in grip position yet")
    elif state == PickSmState.GRASP_OBJECT:
        #print("[SM_INFO] : GRASP_OBJECT")
        pose_pos = wp.transform_get_translation(object_pose[tid])
        pose_rot = wp.transform_get_rotation(object_pose[tid])
        # Apply offset in x-direction (5 cm = 0.05 m) and z direction
        offset_pos = wp.vec3(pose_pos.x , pose_pos.y, pose_pos.z)
        des_ee_pose[tid] = wp.transform(offset_pos, pose_rot)
        gripper_state[tid] = GripperState.CLOSE
        # wait for a while
        if sm_wait_time[tid] >= PickSmWaitTime.GRASP_OBJECT:
            # move to next state and reset wait time
            #print("[SM_INFO] : Moving from GRSP to LIFT_OBJECT")
            sm_state[tid] = PickSmState.LIFT_OBJECT
            sm_wait_time[tid] = 0.0
    elif state == PickSmState.LIFT_OBJECT:
       # print("[SM_INFO] : LIFT_OBJECT")
        pose_pos = wp.transform_get_translation(des_object_pose[tid])
        pose_rot = wp.transform_get_rotation(object_pose[tid])
        offset_pos = wp.vec3(pose_pos.x+0.05 , pose_pos.y+0.01, pose_pos.z+0.01)
        des_ee_pose[tid] = wp.transform(offset_pos, pose_rot)
        gripper_state[tid] = GripperState.CLOSE
        # wait for a while
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            0.05,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickSmWaitTime.LIFT_OBJECT:
                # move to next state and reset wait time
                #print("[SM_INFO] : Moving from LIFT to APPROACH_ABOVE_OBJECT2")
                sm_state[tid] = PickSmState.APPROACH_ABOVE_OBJECT2
                sm_wait_time[tid] = 0.0
    elif state == PickSmState.APPROACH_ABOVE_OBJECT2:
       # print("[SM_INFO] : APPROACH_ABOVE_OBJECT2")
        offset_pos = wp.transform_get_translation(offset[tid])
        offset_rot = wp.transform_get_rotation(offset[tid])
        offset_pos = wp.vec3(offset_pos.x+0.05, offset_pos.y, offset_pos.z + 0.25)  # raise 25 cm 
        new_offset = wp.transform(offset_pos, offset_rot)
        above_target_pose = wp.transform_multiply(new_offset, final_object_pose[tid])
        # Blend time for smooth approach
        APPROACH_BLEND_TIME = 0.4  # seconds, tune as needed
        alpha = wp.clamp(sm_wait_time[tid] / APPROACH_BLEND_TIME, 0.0, 1.0)
        # Interpolate position and rotation
        current_pos = wp.transform_get_translation(ee_pose[tid])
        current_rot = wp.transform_get_rotation(ee_pose[tid])

        target_pos = wp.transform_get_translation(above_target_pose)
        target_rot = wp.transform_get_rotation(above_target_pose)

        pos_interp = wp.lerp(current_pos, target_pos, alpha)
        rot_interp = wp.quat_slerp(current_rot, target_rot, alpha)
        # Set interpolated pose
        des_ee_pose[tid] = wp.transform(pos_interp, rot_interp)
        gripper_state[tid] = GripperState.CLOSE
        # Evaluate readiness to transition
        if distance_below_threshold(current_pos, target_pos, 0.05):
            if sm_wait_time[tid] >= PickSmWaitTime.APPROACH_ABOVE_OBJECT2:
                #print("[SM_INFO] : Moving from APPR_ABOVE to APPROACH_OBJECT2")
                sm_state[tid] = PickSmState.APPROACH_OBJECT2
                sm_wait_time[tid] = 0.0
    elif state == PickSmState.APPROACH_OBJECT2:
        #print("[SM_INFO] : APPROACH_OBJECT2")
        ## Pushes a bit into flask because already on hot plate at this point
        # Get original transform from final_object_pose[tid]
        pose_pos = wp.transform_get_translation(final_object_pose[tid])
        pose_rot = wp.transform_get_rotation(final_object_pose[tid])
        # Apply offset in x-direction (5 cm = 0.05 m)
        offset_pos = wp.vec3(pose_pos.x + 0.05, pose_pos.y, pose_pos.z + 0.1)

        # Reconstruct the transform with new position and same rotation
        des_ee_pose[tid] = wp.transform(offset_pos, pose_rot)
        # des_ee_pose[tid] = final_object_pose[tid]
        gripper_state[tid] = GripperState.CLOSE
        # wait for a while
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            0.02,
        ):
            if sm_wait_time[tid] >= PickSmWaitTime.APPROACH_OBJECT2:
                # move to next state and reset wait time
                #print("[SM_INFO] : Moving from APPROACH_OBJ2 to UNGRASP")
                sm_state[tid] = PickSmState.UNGRASP_OBJECT
                sm_wait_time[tid] = 0.0
            #else:
             #   print("waiting time")
        #e#lse:
             #print("waiting distance")
    elif state == PickSmState.UNGRASP_OBJECT:
        #print("[SM_INFO] : UNGRASP_OBJECT")
        pose_pos = wp.transform_get_translation(final_object_pose[tid])
        pose_rot = wp.transform_get_rotation(final_object_pose[tid])
        # Apply offset in x-direction (5 cm = 0.05 m)
        offset_pos = wp.vec3(pose_pos.x + 0.05, pose_pos.y, pose_pos.z + 0.05)

        # Reconstruct the transform with new position and same rotation
        des_ee_pose[tid] = wp.transform(offset_pos, pose_rot)
        gripper_state[tid] = GripperState.OPEN
        # wait for a while - idk stirring or sthn
        if sm_wait_time[tid] >= PickSmWaitTime.UNGRASP_OBJECT:
            # move to next state and reset wait time
            #print("[SM_INFO] : Moving from UNGRASP to TASK2")
            sm_state[tid] = PickSmState.REGRASP_OBJECT_2
            sm_wait_time[tid] = 0.0
    elif state == PickSmState.REGRASP_OBJECT_2:
        #print("[SM_INFO] : REGRASP_OBJECT")
        ## Pushes a bit into flask because already on hot plate at this point
        # Get original transform from final_object_pose[tid]
        pose_pos = wp.transform_get_translation(final_object_pose[tid])
        pose_rot = wp.transform_get_rotation(final_object_pose[tid])
        # Apply offset in x-direction (5 cm = 0.05 m)
        offset_pos = wp.vec3(pose_pos.x + 0.05, pose_pos.y, pose_pos.z + 0.05)

        # Reconstruct the transform with new position and same rotation
        des_ee_pose[tid] = wp.transform(offset_pos, pose_rot)
        # des_ee_pose[tid] = final_object_pose[tid]
        gripper_state[tid] = GripperState.OPEN
        # wait for a while
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold,
        ):
            if sm_wait_time[tid] >= PickSmWaitTime.REGRASP_OBJECT_2:
                # move to next state and reset wait time
                #print("[SM_INFO] : Moving from APPR_OBJ to GRASP_OBJECT")
                sm_state[tid] = PickSmState.GRASP_OBJECT_2
                sm_wait_time[tid] = 0.0
    elif state == PickSmState.GRASP_OBJECT_2:
        #print("[SM_INFO] : GRASP_OBJECT")
        pose_pos = wp.transform_get_translation(object_pose[tid])
        pose_rot = wp.transform_get_rotation(object_pose[tid])
        # Apply offset in x-direction (5 cm = 0.05 m) and z direction
        offset_pos = wp.vec3(pose_pos.x + 0.02, pose_pos.y, pose_pos.z+0.05)
        des_ee_pose[tid] = wp.transform(offset_pos, pose_rot)
        gripper_state[tid] = GripperState.CLOSE
        if sm_wait_time[tid] >= PickSmWaitTime.GRASP_OBJECT_2:
            # move to next state and reset wait time
            #print("[SM_INFO] : Moving from GRSP to LIFT_OBJECT")
            sm_state[tid] = PickSmState.LIFT_OBJECT_2
            sm_wait_time[tid] = 0.0
    elif state == PickSmState.LIFT_OBJECT_2:
        #print("[SM_INFO] : LIFT_OBJECT2")
        des_ee_pose[tid] = des_object_pose[tid]
        gripper_state[tid] = GripperState.CLOSE
        # wait for a while
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            0.03,
        ):
            # wait for a while
            if sm_wait_time[tid] >= PickSmWaitTime.LIFT_OBJECT_2:
                # move to next state and reset wait time
               # print("[SM_INFO] : Moving from LIFT to APPROACH_ABOVE_OBJECT2")
                sm_state[tid] = PickSmState.APPROACH_ABOVE_OBJECT3
                sm_wait_time[tid] = 0.0
    elif state == PickSmState.APPROACH_ABOVE_OBJECT3:
        #print("[SM_INFO] : APPROACH_ABOVE_OBJECT3")
        offset_pos = wp.transform_get_translation(offset[tid])
        offset_rot = wp.transform_get_rotation(offset[tid])
        offset_pos = wp.vec3(offset_pos.x + 0.05, offset_pos.y, offset_pos.z + 0.25)  # raise 25 cm 
        new_offset = wp.transform(offset_pos, offset_rot)
        above_target_pose = wp.transform_multiply(new_offset, object_task2_pose[tid])
        # Blend time for smooth approach
        APPROACH_BLEND_TIME = 0.4  # seconds, tune as needed
        alpha = wp.clamp(sm_wait_time[tid] / APPROACH_BLEND_TIME, 0.0, 1.0)
        # Interpolate position and rotation
        current_pos = wp.transform_get_translation(ee_pose[tid])
        current_rot = wp.transform_get_rotation(ee_pose[tid])

        target_pos = wp.transform_get_translation(above_target_pose)
        target_rot = wp.transform_get_rotation(above_target_pose)

        pos_interp = wp.lerp(current_pos, target_pos, alpha)
        rot_interp = wp.quat_slerp(current_rot, target_rot, alpha)
        # Set interpolated pose
        des_ee_pose[tid] = wp.transform(pos_interp, rot_interp)
        gripper_state[tid] = GripperState.CLOSE
        # Evaluate readiness to transition
        if distance_below_threshold(current_pos, target_pos, 0.02):
            if sm_wait_time[tid] >= PickSmWaitTime.APPROACH_ABOVE_OBJECT3:
                #print("[SM_INFO] : Moving from APPR_ABOVE to APPROACH_OBJECT2")
                sm_state[tid] = PickSmState.APPROACH_OBJECT3
                sm_wait_time[tid] = 0.0
    elif state == PickSmState.APPROACH_OBJECT3:
       # print("[SM_INFO] : APPROACH_OBJECT3")
        ## Pushes a bit into flask because already on hot plate at this point
        # Get original transform from final_object_pose[tid]
        pose_pos = wp.transform_get_translation(object_task2_pose[tid])
        pose_rot = wp.transform_get_rotation(object_task2_pose[tid])
        # Apply offset in x-direction (5 cm = 0.05 m)
        offset_pos = wp.vec3(pose_pos.x + 0.05, pose_pos.y, pose_pos.z + 0.125)

        # Reconstruct the transform with new position and same rotation
        des_ee_pose[tid] = wp.transform(offset_pos, pose_rot)
        gripper_state[tid] = GripperState.CLOSE
        # wait for a while
        if distance_below_threshold(
            wp.transform_get_translation(ee_pose[tid]),
            wp.transform_get_translation(des_ee_pose[tid]),
            position_threshold,
        ):
            if sm_wait_time[tid] >= PickSmWaitTime.APPROACH_OBJECT3:
                # move to next state and reset wait time
               # print("[SM_INFO] : Moving from APPROACH_OBJ2 to UNGRASP")
                sm_state[tid] = PickSmState.UNGRASP_OBJECT_2
                sm_wait_time[tid] = 0.0
    elif state == PickSmState.UNGRASP_OBJECT_2:
       # print("[SM_INFO] : UNGRASP_OBJECT2")
        pose_pos = wp.transform_get_translation(object_task2_pose[tid])
        pose_rot = wp.transform_get_rotation(object_task2_pose[tid])
        offset_pos = wp.vec3(pose_pos.x + 0.05, pose_pos.y, pose_pos.z + 0.15)
        des_ee_pose[tid] = object_task2_pose[tid]
        gripper_state[tid] = GripperState.OPEN
        # wait for a while
        if sm_wait_time[tid] >= PickSmWaitTime.UNGRASP_OBJECT_2:
            # move to next state and reset wait time
            
            sm_state[tid] = PickSmState.POST_REST
            sm_wait_time[tid] = 0.0
    elif state == PickSmState.POST_REST:
        des_ee_pose[tid] = ee_pose[tid]
        gripper_state[tid] = GripperState.OPEN
        # wait for a while
        if sm_wait_time[tid] >= PickSmWaitTime.POST_REST:
            #print("[SM_INFO] : Moving from REST to APPROACH_ABOVE_OBJECT")
            # move to next state and reset wait time
            #print("[SM_INFO] : DONE SM")
            sm_state[tid] = PickSmState.REST
            sm_wait_time[tid] = 0.0
    # increment wait time
    sm_wait_time[tid] = sm_wait_time[tid] + dt[tid]
    debug_des_pose[tid] = des_ee_pose[tid]
    debug_cur_pose[tid] = ee_pose[tid]


class PickAndLiftSm:
    """A simple state machine in a robot's task space to pick and lift an object.

    The state machine is implemented as a warp kernel. It takes in the current state of
    the robot's end-effector and the object, and outputs the desired state of the robot's
    end-effector and the gripper. The state machine is implemented as a finite state
    machine with the following states:

    1. REST: The robot is at rest.
    2. APPROACH_ABOVE_OBJECT: The robot moves above the object.
    3. APPROACH_OBJECT: The robot moves to the object.
    4. GRASP_OBJECT: The robot grasps the object.
    5. LIFT_OBJECT: The robot lifts the object to the desired pose. This is the final state.
    6. APPROACH_ABOVE_OBJECT2: The robot moves above another object.
    7. APPROACH_OBJECT2: The robot moves to another object.
    8. UNGRASP_OBJECT: The robot ungrasps the object.
    """

    def __init__(self, dt: float, num_envs: int, device: torch.device | str = "cpu", position_threshold=0.01):
        """Initialize the state machine.

        Args:
            dt: The environment time step.
            num_envs: The number of environments to simulate.
            device: The device to run the state machine on.
        """
        # save parameters
        self.dt = float(dt)
        self.num_envs = num_envs
        self.device = device
        self.position_threshold = position_threshold

        # initialize state machine
        self.sm_dt = torch.full((self.num_envs,), self.dt, device=self.device)
        self.sm_state = torch.full((self.num_envs,), 0, dtype=torch.int32, device=self.device)
        self.sm_wait_time = torch.zeros((self.num_envs,), device=self.device)

        # desired state
        self.des_ee_pose = torch.zeros((self.num_envs, 7), device=self.device)
        self.des_gripper_state = torch.full((self.num_envs,), 0.0, device=self.device)
        # self.final_object_pose = torch.zeros((self.num_envs, 7), device=self.device)
        # self.object_task2_pose = torch.zeros((self.num_envs, 7), device=self.device)

        # approach above object offset
        self.offset = torch.zeros((self.num_envs, 7), device=self.device)
        self.offset[:, 2] = 0.15
        self.offset[:, -1] = 1.0  # warp expects quaternion as (x, y, z, w)

        # convert to warp
        self.sm_dt_wp = wp.from_torch(self.sm_dt, wp.float32)
        self.sm_state_wp = wp.from_torch(self.sm_state, wp.int32)
        self.sm_wait_time_wp = wp.from_torch(self.sm_wait_time, wp.float32)
        self.des_ee_pose_wp = wp.from_torch(self.des_ee_pose, wp.transform)
        self.des_gripper_state_wp = wp.from_torch(self.des_gripper_state, wp.float32)
        # self.final_object_pose_wp = wp.from_torch(self.final_object_pose, wp.transform)
        # self.object_task2_pose_wp = wp.from_torch(self.object_task_pose, wp.transform)
        self.offset_wp = wp.from_torch(self.offset, wp.transform)
        ## For Debug 
        self.debug_des_pose = torch.zeros((self.num_envs, 7), device=self.device)
        self.debug_cur_pose = torch.zeros((self.num_envs, 7), device=self.device)

        self.debug_des_pose_wp = wp.from_torch(self.debug_des_pose, wp.transform)
        self.debug_cur_pose_wp = wp.from_torch(self.debug_cur_pose, wp.transform)

    def reset_idx(self, env_ids: Sequence[int] = None):
        """Reset the state machine."""
        if env_ids is None:
            env_ids = slice(None)
        self.sm_state[env_ids] = 0
        self.sm_wait_time[env_ids] = 0.0

    def compute(self, ee_pose: torch.Tensor, object_pose: torch.Tensor, des_object_pose: torch.Tensor, final_object_pose: torch.Tensor, object_task2_pose: torch.Tensor) -> torch.Tensor:
        """Compute the desired state of the robot's end-effector and the gripper."""
        # convert all transformations from (w, x, y, z) to (x, y, z, w)
        ee_pose = ee_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        object_pose = object_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        des_object_pose = des_object_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        final_object_pose = final_object_pose[:, [0, 1, 2, 4, 5, 6, 3]]
        object_task2_pose = object_task2_pose[:, [0, 1, 2, 4, 5, 6, 3]]

        # convert to warp
        ee_pose_wp = wp.from_torch(ee_pose.contiguous(), wp.transform)
        object_pose_wp = wp.from_torch(object_pose.contiguous(), wp.transform)
        des_object_pose_wp = wp.from_torch(des_object_pose.contiguous(), wp.transform)
        final_object_pose_wp = wp.from_torch(final_object_pose.contiguous(), wp.transform)
        object_task2_pose_wp = wp.from_torch(object_task2_pose.contiguous(), wp.transform)
        #print(f"Final object pose : {final_object_pose_wp}")
        # run state machine
        wp.launch(
            kernel=infer_state_machine,
            dim=self.num_envs,
            inputs=[
                self.sm_dt_wp,
                self.sm_state_wp,
                self.sm_wait_time_wp,
                ee_pose_wp,
                object_pose_wp,
                des_object_pose_wp,
                final_object_pose_wp,
                object_task2_pose_wp,
                self.des_ee_pose_wp,
                self.des_gripper_state_wp,
                self.offset_wp,
                self.position_threshold,
                self.debug_des_pose_wp,
                self.debug_cur_pose_wp,
            ],
            device=self.device,
        )
        des_debug = self.debug_des_pose.detach().cpu().numpy()
        cur_debug = self.debug_cur_pose.detach().cpu().numpy()
        
        # for i in range(self.num_envs):
        #     des_pos, des_quat = des_debug[i][:3], des_debug[i][3:]
        #     cur_pos, cur_quat = cur_debug[i][:3], cur_debug[i][3:]
        #     # compute Euclidean distance with numpy
        #     dist = np.linalg.norm(cur_pos - des_pos)
        #     print(f"[Env {i}]")
        #     print(f"   Current -> pos: {cur_pos}, quat: {cur_quat}")
        #     print(f"   Desired -> pos: {des_pos}, quat: {des_quat}")
        #     print(f"   Euclidean distance: {dist:.4f}")
        des_ee_pose = self.des_ee_pose[:, [0, 1, 2, 6, 3, 4, 5]]
        # convert to torch
        return torch.cat([des_ee_pose, self.des_gripper_state.unsqueeze(-1)], dim=-1)


def main():
    # parse configuration
    env_cfg: LiftEnvCfg = parse_env_cfg( # might need to change LiftEnvCfg
        # "Isaac-Stack-Lab-Franka-IK-Abs-v0",
        "Isaac-Stack-LLM-Franka-IK-Abs-v0",
        device=args_cli.device,
        num_envs=args_cli.num_envs,
        use_fabric=not args_cli.disable_fabric,
    )
    # create environment
    # env = gym.make("Isaac-Stack-Lab-Franka-IK-Abs-v0", cfg=env_cfg)
    env = gym.make("Isaac-Stack-LLM-Franka-IK-Abs-v0", cfg=env_cfg)
    # reset environment at start
    env.reset()
   # print("Setup action buffer : shape : ", env.unwrapped.action_space.shape)
    # create action buffers (position + quaternion)
    actions = torch.zeros(env.unwrapped.action_space.shape, device=env.unwrapped.device)
    actions[:, 3] = 1.0
    
   # print("actions : ", actions)
    # desired object orientation (we only do position control of object)
    desired_orientation = torch.zeros((env.unwrapped.num_envs, 4), device=env.unwrapped.device)
    desired_orientation[:, 1] = 1.0 #0?
    # create state machine
    pick_sm = PickAndLiftSm(
        env_cfg.sim.dt * env_cfg.decimation, env.unwrapped.num_envs, env.unwrapped.device, position_threshold=0.05
    )
   # print("debug1 : " , env.unwrapped.action_space.shape)
    n=0
    while simulation_app.is_running():
        # run everything in inference mode+
       # print("loop : ", n)
        with torch.inference_mode():
            # step environment
           # print("debug2 : " , env.unwrapped.action_space.shape)
            dones = env.step(actions)[-2] 
            #print("debug2 : " , env.unwrapped.action_space.shape)
            # observations
            # -- end-effector frame
            ee_frame_sensor = env.unwrapped.scene["ee_frame"]
            tcp_rest_position = ee_frame_sensor.data.target_pos_w[..., 0, :].clone() - env.unwrapped.scene.env_origins
            tcp_rest_orientation = ee_frame_sensor.data.target_quat_w[..., 0, :].clone()
            # -- object frame - changed object -> object1 like my env
            object_data: RigidObjectData = env.unwrapped.scene["object1"].data
            object_position = object_data.root_pos_w - env.unwrapped.scene.env_origins

            final_data: RigidObjectData = env.unwrapped.scene["object2"].data
            final_position = final_data.root_pos_w - env.unwrapped.scene.env_origins

            object_task2_data: RigidObjectData = env.unwrapped.scene["object3"].data
            object_task2_position = object_task2_data.root_pos_w - env.unwrapped.scene.env_origins

            # -- target object frame
            desired_position = env.unwrapped.command_manager.get_command("object_pose")[..., :3]

            # advance state machine
            actions = pick_sm.compute(
                torch.cat([tcp_rest_position, tcp_rest_orientation], dim=-1),
                torch.cat([object_position, tcp_rest_orientation], dim=-1),
                torch.cat([desired_position, tcp_rest_orientation], dim=-1),
                torch.cat([final_position, tcp_rest_orientation], dim=-1),
                torch.cat([object_task2_position, tcp_rest_orientation], dim=-1),
            )
        
            
           # print(f"End of loop : {actions.shape}")
            #print("Action space shape:", env.unwrapped.action_space.shape)
            

            
           
            # reset state machine
            if dones.any():
                pick_sm.reset_idx(dones.nonzero(as_tuple=False).squeeze(-1))
        # Step counter
        n=n+1    
    # close the environment
    env.close()


if __name__ == "__main__":

    main()
    # close sim app
    simulation_app.close()