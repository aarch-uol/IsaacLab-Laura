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

"""Configuration for the Ridgeback-Manipulation robots.

The following configurations are available:

* :obj:`RIDGEBACK_FRANKA_PANDA_CFG`: Clearpath Ridgeback base with Franka Emika arm

Reference: https://github.com/ridgeback/ridgeback_manipulation
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Configuration
##

RIDGEBACK_FRANKA_PANDA_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Clearpath/RidgebackFranka/ridgeback_franka.usd",
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=False),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # base
            "dummy_base_prismatic_y_joint": 0.0,
            "dummy_base_prismatic_x_joint": 0.0,
            "dummy_base_revolute_z_joint": 0.0,
            # franka arm
            "panda_joint1": 0.0,
            "panda_joint2": -0.569,
            "panda_joint3": 0.0,
            "panda_joint4": -2.810,
            "panda_joint5": 0.0,
            "panda_joint6": 2.0,
            "panda_joint7": 0.741,
            # tool
            "panda_finger_joint.*": 0.035,
        },
        joint_vel={".*": 0.0},
    ),
    actuators={
        "base": ImplicitActuatorCfg(
            joint_names_expr=["dummy_base_.*"],
<<<<<<< HEAD
<<<<<<< HEAD
            effort_limit_sim=1000.0,
=======
            velocity_limit=100.0,
            effort_limit=1000.0,
>>>>>>> abfba5273e (Fresh start, no history)
=======
            velocity_limit=100.0,
            effort_limit=1000.0,
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
            stiffness=0.0,
            damping=1e5,
        ),
        "panda_shoulder": ImplicitActuatorCfg(
            joint_names_expr=["panda_joint[1-4]"],
<<<<<<< HEAD
<<<<<<< HEAD
            effort_limit_sim=87.0,
=======
            effort_limit=87.0,
            velocity_limit=100.0,
>>>>>>> abfba5273e (Fresh start, no history)
=======
            effort_limit=87.0,
            velocity_limit=100.0,
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
            stiffness=800.0,
            damping=40.0,
        ),
        "panda_forearm": ImplicitActuatorCfg(
            joint_names_expr=["panda_joint[5-7]"],
<<<<<<< HEAD
<<<<<<< HEAD
            effort_limit_sim=12.0,
=======
            effort_limit=12.0,
            velocity_limit=100.0,
>>>>>>> abfba5273e (Fresh start, no history)
=======
            effort_limit=12.0,
            velocity_limit=100.0,
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
            stiffness=800.0,
            damping=40.0,
        ),
        "panda_hand": ImplicitActuatorCfg(
            joint_names_expr=["panda_finger_joint.*"],
<<<<<<< HEAD
<<<<<<< HEAD
            effort_limit_sim=200.0,
=======
            effort_limit=200.0,
            velocity_limit=0.2,
>>>>>>> abfba5273e (Fresh start, no history)
=======
            effort_limit=200.0,
            velocity_limit=0.2,
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
            stiffness=1e5,
            damping=1e3,
        ),
    },
)
"""Configuration of Franka arm with Franka Hand on a Clearpath Ridgeback base using implicit actuator models.

The following control configuration is used:

* Base: velocity control
* Arm: position control with damping
* Hand: position control with damping

"""
