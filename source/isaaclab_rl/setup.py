# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Installation script for the 'isaaclab_rl' python package."""

import itertools
import os
import toml

from setuptools import setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

INSTALL_REQUIRES = [
    # generic
    "numpy<2",  # Pin to numpy 1.x for Isaac compatibility; remove '<2' if you want to support numpy 2.x
    "torch==2.5.1",
    "torchvision>=0.14.1",  # ensure compatibility with torch 1.13.1
    "protobuf>=3.20.2,!=5.26.0",
    # configuration management
    "hydra-core",
    # data collection
    "h5py",
    # basic logger
    "tensorboard",
    # video recording
    "moviepy",
    # make sure this is consistent with isaac sim version
    "pillow==11.2.1",  # adjust as needed for your stack
]

PYTORCH_INDEX_URL = ["https://download.pytorch.org/whl/cu118"]

EXTRAS_REQUIRE = {
    "sb3": ["stable-baselines3>=2.6", "tqdm", "rich"],  # tqdm/rich for progress bar
    "skrl": ["skrl>=1.4.2"],
    "rl-games": ["rl-games==1.6.1", "gym"],  # rl-games still needs gym :(
    "rsl-rl": ["rsl-rl-lib==2.3.3"],
}
# Add alias keys (underscores and hyphens)
EXTRAS_REQUIRE["rl_games"] = EXTRAS_REQUIRE["rl-games"]
EXTRAS_REQUIRE["rsl_rl"] = EXTRAS_REQUIRE["rsl-rl"]

# All-in-one key with deduplication
EXTRAS_REQUIRE["all"] = list(set(itertools.chain.from_iterable(EXTRAS_REQUIRE.values())))

setup(
    name="isaaclab_rl",
    author="Isaac Lab Project Developers",
    maintainer="Isaac Lab Project Developers",
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    include_package_data=True,
    python_requires=">=3.10",
    install_requires=INSTALL_REQUIRES,
    dependency_links=PYTORCH_INDEX_URL,
    extras_require=EXTRAS_REQUIRE,
    packages=["isaaclab_rl"],
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.10",
        "Isaac Sim :: 4.5.0",
    ],
    zip_safe=False,
)
