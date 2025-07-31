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

import argparse

import pytest

from isaaclab.app import AppLauncher


@pytest.mark.usefixtures("mocker")
def test_livestream_launch_with_argparser(mocker):
    """Test launching with argparser arguments."""
    # Mock the parse_args method
    mocker.patch("argparse.ArgumentParser.parse_args", return_value=argparse.Namespace(livestream=1, headless=True))
    # create argparser
    parser = argparse.ArgumentParser()
    # add app launcher arguments
    AppLauncher.add_app_launcher_args(parser)
    # check that argparser has the mandatory arguments
    for name in AppLauncher._APPLAUNCHER_CFG_INFO:
        assert parser._option_string_actions[f"--{name}"]
    # parse args
    mock_args = parser.parse_args()
    # everything defaults to None
    app = AppLauncher(mock_args).app

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
