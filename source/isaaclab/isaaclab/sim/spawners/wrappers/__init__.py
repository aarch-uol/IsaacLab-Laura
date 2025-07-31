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

"""Sub-module for wrapping spawner configurations.

Unlike the other spawner modules, this module provides a way to wrap multiple spawner configurations
into a single configuration. This is useful when the user wants to spawn multiple assets based on
different configurations.
"""

from .wrappers import spawn_multi_asset, spawn_multi_usd_file
from .wrappers_cfg import MultiAssetSpawnerCfg, MultiUsdFileCfg
