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

"""Sub-package with utilities for creating terrains procedurally.

There are two main components in this package:

* :class:`TerrainGenerator`: This class procedurally generates terrains based on the passed
  sub-terrain configuration. It creates a ``trimesh`` mesh object and contains the origins of
  each generated sub-terrain.
* :class:`TerrainImporter`: This class mainly deals with importing terrains from different
  possible sources and adding them to the simulator as a prim object.
  The following functions are available for importing terrains:

  * :meth:`TerrainImporter.import_ground_plane`: spawn a grid plane which is default in Isaac Sim.
  * :meth:`TerrainImporter.import_mesh`: spawn a prim from a ``trimesh`` object.
  * :meth:`TerrainImporter.import_usd`: spawn a prim as reference to input USD file.

"""
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> e9462be776417c5794982ad017c44c19fac790a2
from .height_field import *  # noqa: F401, F403
from .terrain_generator import TerrainGenerator
<<<<<<< HEAD
from .terrain_generator_cfg import TerrainGeneratorCfg
=======
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5

from .height_field import *  # noqa: F401, F403
from .terrain_generator import TerrainGenerator
from .terrain_generator_cfg import FlatPatchSamplingCfg, SubTerrainBaseCfg, TerrainGeneratorCfg
<<<<<<< HEAD
>>>>>>> abfba5273e (Fresh start, no history)
=======
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
=======
from .terrain_generator_cfg import FlatPatchSamplingCfg, SubTerrainBaseCfg, TerrainGeneratorCfg
>>>>>>> e9462be776417c5794982ad017c44c19fac790a2
from .terrain_importer import TerrainImporter
from .terrain_importer_cfg import TerrainImporterCfg
from .trimesh import *  # noqa: F401, F403
from .utils import color_meshes_by_height, create_prim_from_mesh
