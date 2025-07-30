<<<<<<< HEAD
<<<<<<< HEAD
# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e (Fresh start, no history)
=======
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
>>>>>>> abfba5273e35ca74eb713aa9a0404a6fad7fd5a5
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import pytest

from isaaclab.app import AppLauncher


@pytest.mark.usefixtures("mocker")
def test_livestream_launch_with_kwargs(mocker):
    """Test launching with keyword arguments."""
    # everything defaults to None
    app = AppLauncher(headless=True, livestream=1).app

    # import settings
    import carb

    # acquire settings interface
    carb_settings_iface = carb.settings.get_settings()
    # check settings
    # -- no-gui mode
    assert carb_settings_iface.get("/app/window/enabled") is False
    # -- livestream
    assert carb_settings_iface.get("/app/livestream/enabled") is True

    # close the app on exit
    app.close()
