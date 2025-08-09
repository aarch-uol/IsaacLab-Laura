
import argparse
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="")
def args_parser(parser, args: list[str] | list[dict]) -> argparse.ArgumentParser:
    """
    Adds arguments to the parser based on ARGS keys or inline dicts.
    Returns the parser (so caller can add more arguments before parsing).
    """
    for key in args:
        if isinstance(key, dict):
            parser.add_argument(*key["flags"], **key["kwargs"])
        elif key in ARGS:
            arg = ARGS[key]
            parser.add_argument(*arg["flags"], **arg["kwargs"])
        else:
            raise ValueError(f"Unknown argument: {key}")

    return parser


# ------------------------------------------------------------------------------------------------------------
# Common AppLauncher ARGS

ARGS = { 
    # region Common Args
    "task": {
        "flags": ("--task",),
        "kwargs": {"type": str, "default": None, "help": "Task name."}
    },
    "num_envs": {
        "flags": ("--num_envs",),
        "kwargs": {"type": int, "default": 1, "help": "Number of environments."}
    },
    "pinocchio": {
        "flags": ("--enable_pinocchio",),
        "kwargs": {"action": "store_true", "default": False, "help": "Enable Pinocchio."}
    },

    # region Telop :
    "teleop_device": {
        "flags": ("--teleop_device",),
        "kwargs": {"type": str, "default": "keyboard", "help": "Device for interacting with environment.",
                    "choices" : ["keyboard", "spacemouse", "gamepad", "handtracking"],
                   }
    },
    "sensitivity": {
        "flags": ("--sensitivity",),
        "kwargs": {"type": float, "default": 1.0, "help": "Sensitivity factor."}
    },

    # region File Handler :
    "dataset_file": {
        "flags": ("--dataset_file",),
        "kwargs": {"type": str, "default": "datasets/dataset.hdf5", "help": "Dataset file to be replayed."}
    },
    "input_file": {
        "flags": ("--input_file",),
        "kwargs": {"type": str, "required": True, "help": "Path to source dataset file."}
    },
    "output_file": {
        "flags": ("--output_file",),
        "kwargs": {"type": str, "required": True, "help": "Path to export output dataset."}
    },

    # region Record :
    "num_demos": {
        "flags": ("--num_demos",),
        "kwargs": {"type": int, "default": 0, "help": "Number of demonstrations to record. Set to 0 for infinite."}
    },
    "num_success_steps": {
        "flags": ("--num_success_steps",),
        "kwargs": {"type": int, "default": 10, "help": (
            "Number of continuous steps with task success for concluding a demo as successful. Default is 10."
        )}
    },
    "step_hz": {
        "flags": ("--step_hz",),
        "kwargs": {"type": int, "default": 30, "help": "Environment stepping rate in Hz."}
    },

    # region Annotate :
    "auto": {
        "flags": ("--auto",),
        "kwargs": {"action": "store_true", "default": False, "help": "Automatically annotate subtasks." }
    },

    # region Generate :
    "gen_nums": {
        "flags": ("--generation_num_trials",),
        "kwargs": {"type": int, "default": None, "help": "Number of demos to generate."}
    },
    "pause_subtask": {
        "flags": ("--pause_subtask",),
        "kwargs": {"action": "store_true", "help": "Pause after every subtask (debugging; use with render flag)."}
    },


    # region replay:
    "select_episodes": {
        "flags": ("--select_episodes",),
        "kwargs": {"type": list, "nargs": "+", "default": [], "help": "Empty list replays all eps."}
    },

    "validate_states": {
        "flags": ("--validate_states",),
        "kwargs": {"action": "store_true", "default": False, "help": (
            "Validate if the states, if available, match between loaded from datasets and replayed. Only valid if --num_envs is 1."
        )}
    },


    # region Consolidate (unused)
    "generated_output_file": {
        "flags": ("--generated_output_file",),
        "kwargs": {"type": str, "default": None, "help": "File path to export generated episodes by mimic."}
    },
    "teleop_env_index": {
        "flags": ("--teleop_env_index",),
        "kwargs": {"type": int, "default": 0, "help": (
            "Index of the environment to be used for teleoperation. Set -1 for disabling the teleop robot. Default is 0."
        )}
    },

}