#!/bin/bash

# This script is a convenient wrapper to run the point cloud rounding
# Python script on a specified folder.

UTILS_DIR="/workspace/utils"
PYTHON_SCRIPT="$UTILS_DIR/clean_point_cloud.py"

# 1. Check for exactly one argument.
if [ $# -ne 1 ]; then
    echo "Usage: $0 path/to/your/processed_data_folder"
    echo "Example: $0 processed_data/rcpcc/training"
    exit 1
fi

FOLDER_PATH="$1"

# 2. Check if the provided path is a directory.
if [ ! -d "$FOLDER_PATH" ]; then
    echo "Error: The provided path is not a valid directory."
    echo "Path: $FOLDER_PATH"
    exit 1
fi

# 3. Check if the python script exists
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: The Python script was not found at: $PYTHON_SCRIPT"
    exit 1
fi

# 4. Run the Python script with the folder path.
echo "--- Starting Point Cloud Processing ---"
python3 "$PYTHON_SCRIPT" "$FOLDER_PATH"
echo "--- Processing Finished ---"
