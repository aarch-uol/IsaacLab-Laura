import h5py

def clean_and_renumber_hdf5(h5_path: str, output_path: str | None = None):
    """
    Removes /data/demo_x groups without datagen_info and renumbers remaining ones
    to keep 0..n order inside an HDF5 file.

    Args:
        h5_path (str): Path to the input HDF5 file.
        output_path (str|None): Path to the cleaned HDF5 file. If None, overwrite input.
    """
    if output_path is None:
        output_path = h5_path + ".cleaned"

    with h5py.File(h5_path, "r") as src, h5py.File(output_path, "w") as dst:
        # Copy everything at root except /data
        for key in src.keys():
            if key != "data":
                src.copy(key, dst)

        # Create /data in the new file
        dst_data = dst.create_group("data")
        src_data = src["data"]

        # Collect valid demo groups
        valid_demos = []
        for key in sorted(src_data.keys(), key=lambda k: int(k.split("_")[1]) if k.startswith("demo_") else 1e9):
            if not key.startswith("demo_"):
                # copy non-demo things under /data directly
                src_data.copy(key, dst_data)
                continue

            grp = src_data[key]
            if "actions" in grp:
                valid_demos.append(key)
            else:
                print(f"Removing /data/{key} (no datagen_info)")

        # Copy valid demos with renumbering
        for new_idx, old_key in enumerate(valid_demos):
            new_key = f"demo_{new_idx}"
            print(f"Copying /data/{old_key} -> /data/{new_key}")
            src_data.copy(old_key, dst_data, name=new_key)

    print(f"✅ Cleaned HDF5 written to {output_path}")

clean_and_renumber_hdf5("docs/training_runs/datasets/weigh_stir_manual_annotated.hdf5", "docs/training_runs/datasets/weigh_stir_manual_annotated_clean.hdf5")