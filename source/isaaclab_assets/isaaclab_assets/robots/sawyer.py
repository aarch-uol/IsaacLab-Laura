<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e (Fresh start, no history)
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> e9462be776417c5794982ad017c44c19fac790a2
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Rethink Robotics arms.

The following configuration parameters are available:

* :obj:`SAWYER_CFG`: The Sawyer arm without any tool attached.

Reference: https://github.com/RethinkRobotics/sawyer_robot
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Configuration
##

SAWYER_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/RethinkRobotics/sawyer_instanceable.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "head_pan": 0.0,
            "right_j0": 0.0,
            "right_j1": -0.785,
            "right_j2": 0.0,
            "right_j3": 1.05,
            "right_j4": 0.0,
            "right_j5": 1.3,
            "right_j6": 0.0,
        },
    ),
    actuators={
        "head": ImplicitActuatorCfg(
            joint_names_expr=["head_pan"],
<<<<<<< HEAD
<<<<<<< HEAD
            effort_limit_sim=8.0,
=======
            velocity_limit=100.0,
            effort_limit=8.0,
>>>>>>> abfba5273e (Fresh start, no history)
=======
            velocity_limit=100.0,
            effort_limit=8.0,
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
            stiffness=800.0,
            damping=40.0,
        ),
        "arm": ImplicitActuatorCfg(
            joint_names_expr=["right_j[0-6]"],
<<<<<<< HEAD
<<<<<<< HEAD
            effort_limit_sim={
=======
            velocity_limit=100.0,
            effort_limit={
>>>>>>> abfba5273e (Fresh start, no history)
=======
            velocity_limit=100.0,
            effort_limit={
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
                "right_j[0-1]": 80.0,
                "right_j[2-3]": 40.0,
                "right_j[4-6]": 9.0,
            },
            stiffness=100.0,
            damping=4.0,
        ),
    },
)
"""Configuration of Rethink Robotics Sawyer arm."""
