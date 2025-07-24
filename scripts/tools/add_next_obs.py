import h5py
import numpy as np
import os
import json # Not directly used for copying, but good to keep for context of env_args

def _deep_copy_hdf5_item(source_item, dest_group, item_name):
    """
    Recursively copies an HDF5 group or dataset, including its attributes.
    """
    if isinstance(source_item, h5py.Group):
        # Create a new group in the destination
        new_group = dest_group.create_group(item_name)
        # Copy attributes
        for attr_key, attr_value in source_item.attrs.items():
            new_group.attrs[attr_key] = attr_value
        # Recursively copy contents
        for key in source_item.keys():
            _deep_copy_hdf5_item(source_item[key], new_group, key)
    elif isinstance(source_item, h5py.Dataset):
        # Create a new dataset in the destination
        new_dataset = dest_group.create_dataset(item_name, data=source_item[()], dtype=source_item.dtype)
        # Copy attributes
        for attr_key, attr_value in source_item.attrs.items():
            new_dataset.attrs[attr_key] = attr_value
    else:
        print(f"Warning: Skipping unsupported HDF5 item type: {type(source_item)} for item '{item_name}'")


def add_next_obs_to_hdf5(input_hdf5_path, output_hdf5_path):
    """
    Creates an exact duplicate of a Robomimic-style HDF5 dataset and then
    adds the 'next_obs' group to each demonstration.

    For each episode, 'next_obs' at time step t is derived from 'obs' at time step t+1.
    The 'next_obs' for the last time step of an episode is a copy of its 'obs'.

    Args:
        input_hdf5_path (str): Path to the original HDF5 dataset file.
        output_hdf5_path (str): Path where the new HDF5 dataset with 'next_obs' will be saved.
    """
    if not os.path.exists(input_hdf5_path):
        print(f"Error: Input HDF5 file not found at '{input_hdf5_path}'")
        return

    if os.path.exists(output_hdf5_path):
        response = input(f"Output file '{output_hdf5_path}' already exists. Overwrite? (y/n): ")
        if response.lower() != 'y':
            print("Operation cancelled by user.")
            return

    print(f"Loading data from: {input_hdf5_path}")
    print(f"Saving processed data to: {output_hdf5_path}")

    try:
        with h5py.File(input_hdf5_path, 'r') as in_f:
            with h5py.File(output_hdf5_path, 'w') as out_f:
                print("Performing deep copy of the entire HDF5 file...")
                # Deep copy the entire input file to the output file
                for key in in_f.keys():
                    _deep_copy_hdf5_item(in_f[key], out_f, key)
                print("Deep copy complete.")

                # Now, iterate through the copied 'data' group to add 'next_obs'
                if 'data' not in out_f:
                    print("Error: '/data' group not found in the copied file. Cannot add 'next_obs'.")
                    return

                data_group_out = out_f['data']
                demo_keys = [k for k in data_group_out.keys() if k.startswith('demo_')]
                print(f"Found {len(demo_keys)} demonstration episodes in the copied file.")

                for i, demo_key in enumerate(demo_keys):
                    print(f"Generating 'next_obs' for episode: {demo_key} ({i+1}/{len(demo_keys)})")
                    demo_group_out = data_group_out[demo_key]

                    if 'obs' not in demo_group_out:
                        print(f"Warning: 'obs' group not found in {demo_key}. Skipping next_obs generation for this episode.")
                        continue

                    obs_group_out = demo_group_out['obs']

                    # Check if 'next_obs' already exists (e.g., from original data) and delete it
                    if 'next_obs' in demo_group_out:
                        del demo_group_out['next_obs']
                        print(f"  Existing 'next_obs' group in {demo_key} removed before regeneration.")

                    next_obs_group_out = demo_group_out.create_group('next_obs')

                    # Iterate through each observation modality (e.g., 'eef_pos', 'rgb', etc.)
                    for obs_key in obs_group_out.keys():
                        obs_data = obs_group_out[obs_key][()] # Load all data for this observation key
                        
                        if obs_data.ndim == 0: # Handle scalar observations (unlikely for robomimic)
                            next_obs_data = obs_data
                        elif obs_data.shape[0] > 0:
                            # Shift observations by one for next_obs
                            # next_obs[t] = obs[t+1]
                            # next_obs[last_step] = obs[last_step]
                            next_obs_data = np.concatenate((obs_data[1:], obs_data[-1:]), axis=0)
                        else: # Empty observation data
                            next_obs_data = obs_data # Keep it empty

                        next_obs_group_out.create_dataset(obs_key, data=next_obs_data, dtype=obs_data.dtype)

        print(f"\nSuccessfully created exact duplicate and added 'next_obs' to: {output_hdf5_path}")

    except Exception as e:
        print(f"An error occurred: {e}")
        print("Please ensure the input HDF5 file has the expected Robomimic structure (e.g., 'data/demo_X/obs/').")

if __name__ == "__main__":
    # --- Configuration ---
    # IMPORTANT: Replace these paths with your actual file paths
    # The input file path should be your existing HDF5 dataset
    INPUT_HDF5_FILE = "/workspace/isaaclab/docs/varying_beaker_scale_generated2_split.hdf5"
    # The output file path will be the new dataset with next_obs added
    # It's recommended to use a new name to avoid overwriting your original file
    OUTPUT_HDF5_FILE = "/workspace/isaaclab/docs/varying_beaker_scale_generated2_split_with_next_obs.hdf5"
    # -------------------

    add_next_obs_to_hdf5(INPUT_HDF5_FILE, OUTPUT_HDF5_FILE)
