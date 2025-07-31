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

"""Configuration for a simple inverted Double Pendulum on a Cart robot."""


import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

CART_DOUBLE_PENDULUM_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Classic/CartDoublePendulum/cart_double_pendulum.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 2.0), joint_pos={"slider_to_cart": 0.0, "cart_to_pole": 0.0, "pole_to_pendulum": 0.0}
    ),
    actuators={
        "cart_actuator": ImplicitActuatorCfg(
            joint_names_expr=["slider_to_cart"],
<<<<<<< HEAD
<<<<<<< HEAD
            effort_limit_sim=400.0,
=======
            effort_limit=400.0,
            velocity_limit=100.0,
>>>>>>> abfba5273e (Fresh start, no history)
=======
            effort_limit=400.0,
            velocity_limit=100.0,
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
            stiffness=0.0,
            damping=10.0,
        ),
        "pole_actuator": ImplicitActuatorCfg(
<<<<<<< HEAD
<<<<<<< HEAD
            joint_names_expr=["cart_to_pole"], effort_limit_sim=400.0, stiffness=0.0, damping=0.0
        ),
        "pendulum_actuator": ImplicitActuatorCfg(
            joint_names_expr=["pole_to_pendulum"], effort_limit_sim=400.0, stiffness=0.0, damping=0.0
=======
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
            joint_names_expr=["cart_to_pole"], effort_limit=400.0, velocity_limit=100.0, stiffness=0.0, damping=0.0
        ),
        "pendulum_actuator": ImplicitActuatorCfg(
            joint_names_expr=["pole_to_pendulum"], effort_limit=400.0, velocity_limit=100.0, stiffness=0.0, damping=0.0
<<<<<<< HEAD
>>>>>>> abfba5273e (Fresh start, no history)
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
        ),
    },
)
"""Configuration for a simple inverted Double Pendulum on a Cart robot."""
