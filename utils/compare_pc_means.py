import numpy as np
import sys
import os

def read_kitti_bin(bin_path):
    point_cloud = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return point_cloud

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compare_pc_means.py <path_to_raw.bin> <path_to_processed.bin>")
        sys.exit(1)

    raw_file = sys.argv[1]
    processed_file = sys.argv[2]

    if not os.path.isfile(raw_file):
        print(f"Error: Raw file not found at '{raw_file}'")
        sys.exit(1)
        
    if not os.path.isfile(processed_file):
        print(f"Error: Processed file not found at '{processed_file}'")
        sys.exit(1)

    raw_points = read_kitti_bin(raw_file)
    processed_points = read_kitti_bin(processed_file)

    raw_mean = np.mean(raw_points[:, :3], axis=0)
    processed_mean = np.mean(processed_points[:, :3], axis=0)

    print(f"--- Point Cloud Mean Comparison ---")
    print(f"Raw file:       {raw_file}")
    print(f"Processed file: {processed_file}")
    print(f"\nRaw point cloud mean (X, Y, Z):       {raw_mean}")
    print(f"Processed point cloud mean (X, Y, Z): {processed_mean}")

    diff = raw_mean - processed_mean
    print(f"\nDifference (Raw - Processed):         {diff}")

    if np.allclose(raw_mean, processed_mean):
        print("\nConclusion: The point clouds have the same center. No translation detected.")
    else:
        print("\nConclusion: The point clouds have different centers. A translation was likely applied during processing.")
