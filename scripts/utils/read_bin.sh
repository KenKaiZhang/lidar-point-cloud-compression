#!/bin/bash

HOME_DIR="/workspace"
UTILS_DIR="$HOME_DIR/utils"

# Ensure a file path is provided as an argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <path_to_file.bin>"
    exit 1
fi

BINARY_FILE="$1"
PYTHON_SCRIPT="$UTILS_DIR/read_bin.py"

# Check if the binary file exists
if [ ! -f "$BINARY_FILE" ]; then
    echo "Error: File not found at '$BINARY_FILE'"
    exit 1
fi

# Check if the python script exists in the same directory
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Python script '$PYTHON_SCRIPT' not found."
    echo "Make sure it is in the same directory as this bash script."
    exit 1
fi

# Execute the python script with the binary file as an argument
# We use python3 as it is the standard now.
echo "Reading binary file: $BINARY_FILE"
echo "---------------------------------------------------"
python3 "$PYTHON_SCRIPT" "$BINARY_FILE"
echo "---------------------------------------------------"
echo "Done."