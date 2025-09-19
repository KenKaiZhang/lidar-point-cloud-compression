import numpy as np
import argparse
import os
import sys
from tqdm import tqdm

def round_point_cloud_file(file_path, decimals=3):
    """
    Reads a KITTI .bin file, rounds the x, y, z coordinates, and overwrites the file.

    Args:
        file_path (str): The full path to the .bin file.
        decimals (int): The number of decimal places to round to.
    """
    try:
        # Read the binary file into an (N, 4) numpy array
        point_cloud = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)

        # Round the first 3 columns (x, y, z) in-place
        np.round(point_cloud[:, :3], decimals, out=point_cloud[:, :3])

        # Overwrite the original file with the rounded data
        point_cloud.astype(np.float32).tofile(file_path)

        return True
    except Exception as e:
        # tqdm doesn't print well with concurrent prints, so we write to stderr
        tqdm.write(f"  └── Error processing file {os.path.basename(file_path)}: {e}", file=sys.stderr)
        return False

def main():
    parser = argparse.ArgumentParser(
        description="Process a folder of .bin files to round point cloud coordinates.",
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument(
        'folder_path',
        type=str,
        help="Path to the folder containing .bin point cloud files.\nThis will search recursively through all subdirectories."
    )
    parser.add_argument(
        '--decimals',
        type=int,
        default=3,
        help="Number of decimal places to round the coordinates to (default: 3)."
    )
    args = parser.parse_args()

    if not os.path.isdir(args.folder_path):
        print(f"Error: Provided path is not a valid directory: {args.folder_path}", file=sys.stderr)
        sys.exit(1)

    print(f"Starting scan to find .bin files in: {args.folder_path}")

    # --- TQDM Integration ---
    # 1. First, collect a list of all files to be processed.
    bin_files = []
    for root, _, files in os.walk(args.folder_path):
        for filename in files:
            if filename.endswith('.bin'):
                bin_files.append(os.path.join(root, filename))
    
    if not bin_files:
        print("No .bin files found in the specified directory.")
        return

    print(f"Found {len(bin_files)} files. Rounding x, y, z coordinates to {args.decimals} decimal places.")
    
    error_count = 0
    # 2. Iterate over the list with tqdm to create the progress bar.
    for file_path in tqdm(bin_files, desc="Processing files"):
        if not round_point_cloud_file(file_path, args.decimals):
            error_count += 1
    # --- End TQDM Integration ---

    print("-" * 40)
    print("Scan complete.")
    print(f"Processed files: {len(bin_files) - error_count}")
    if error_count > 0:
        print(f"Files with errors: {error_count}")
    else:
        print("No errors encountered.")

if __name__ == "__main__":
    main()

