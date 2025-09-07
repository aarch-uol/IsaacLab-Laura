import h5py

def remove_demo(h5_path: str, demo_key: str = "demo_0"):
    """
    Remove a specific demo group (e.g., /data/demo_0) from an HDF5 file.
    
    Args:
        h5_path (str): Path to the HDF5 file.
        demo_key (str): Name of the demo group to remove (e.g., "demo_0").
    """
    with h5py.File(h5_path, "a") as f:
        if "data" in f and demo_key in f["data"]:
            print(f"Removing /data/{demo_key}")
            del f["data"][demo_key]
        else:
            print(f"/data/{demo_key} not found — nothing removed.")

# Example usage:
remove_demo("docs/training_runs/datasets/weigh_stir_manual_annotated.hdf5", "demo_0")
