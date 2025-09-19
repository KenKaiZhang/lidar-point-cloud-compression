import numpy as np
import sys
import os

def read_kitti_bin(bin_path):
    """
    Reads a KITTI .bin file and returns an Nx4 numpy array.
    Each row is [x, y, z, reflectance].
    """
    point_cloud = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return point_cloud

if __name__ == "__main__":
    # Ensure the script is called with one argument
    if len(sys.argv) != 2:
        print("Usage: python read_bin.py <path_to_file.bin>")
        sys.exit(1)

    bin_file = sys.argv[1]

    # Validate file exists
    if not os.path.isfile(bin_file):
        print(f"Error: File not found at '{bin_file}'")
        sys.exit(1)

    # Read and display information
    points = read_kitti_bin(bin_file)
    print(f"Loaded {points.shape[0]} points from '{bin_file}'")
    print("First 5 points (x, y, z, reflectance):")
    for point in points:
        print(point)
    # print(points[:5])