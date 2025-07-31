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

from isaaclab.sim.converters.asset_converter_base_cfg import AssetConverterBaseCfg
from isaaclab.sim.schemas import schemas_cfg
from isaaclab.utils import configclass


@configclass
class MeshConverterCfg(AssetConverterBaseCfg):
    """The configuration class for MeshConverter."""

    mass_props: schemas_cfg.MassPropertiesCfg | None = None
    """Mass properties to apply to the USD. Defaults to None.

    Note:
        If None, then no mass properties will be added.
    """

    rigid_props: schemas_cfg.RigidBodyPropertiesCfg | None = None
    """Rigid body properties to apply to the USD. Defaults to None.

    Note:
        If None, then no rigid body properties will be added.
    """

    collision_props: schemas_cfg.CollisionPropertiesCfg | None = None
    """Collision properties to apply to the USD. Defaults to None.

    Note:
        If None, then no collision properties will be added.
    """

    collision_approximation: str = "convexDecomposition"
    """Collision approximation method to use. Defaults to "convexDecomposition".

    Valid options are:
    "convexDecomposition", "convexHull", "boundingCube",
    "boundingSphere", "meshSimplification", or "none"

    "none" causes no collision mesh to be added.
    """

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """The translation of the mesh to the origin. Defaults to (0.0, 0.0, 0.0)."""

    rotation: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)
    """The rotation of the mesh in quaternion format (w, x, y, z). Defaults to (1.0, 0.0, 0.0, 0.0)."""

    scale: tuple[float, float, float] = (1.0, 1.0, 1.0)
    """The scale of the mesh. Defaults to (1.0, 1.0, 1.0)."""
