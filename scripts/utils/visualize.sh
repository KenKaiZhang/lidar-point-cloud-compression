#!/bin/bash

# This script simplifies the comparison of raw and processed point clouds.
# It takes a single file ID (e.g., 000000) and constructs the necessary paths.

# --- Configuration ---
RAW_DATA_DIR="datasets/training"
PROCESSED_DATA_DIR="processed_data/rcpcc/training"
UTILS_DIR="/workspace/utils"
# --- End Configuration ---


# 1. Check for exactly one argument.
if [ $# -ne 1 ]; then
    echo "Usage: $0 <file_id>"
    echo "Example: $0 000000"
    exit 1
fi

FILE_ID="$1"

# 2. Construct the full paths for all required files.
RAW_BIN_FILE="${RAW_DATA_DIR}/velodyne/${FILE_ID}.bin"
PROCESSED_BIN_FILE="${PROCESSED_DATA_DIR}/velodyne/${FILE_ID}.bin"
LABEL_FILE="${RAW_DATA_DIR}/label_2/${FILE_ID}.txt" # Assumes standard KITTI label path
OUTPUT_FILE="comparison_${FILE_ID}.html"

# 3. Verify that all input files exist before running the script.
for f in "$RAW_BIN_FILE" "$PROCESSED_BIN_FILE" "$LABEL_FILE"; do
    if [ ! -f "$f" ]; then
        echo "Error: Required input file not found at: $f"
        exit 1
    fi
done

# 4. Run the Python script with all four constructed arguments.
echo "--- Starting Comparison for ${FILE_ID} ---"
echo "Raw data:       $RAW_BIN_FILE"
echo "Processed data: $PROCESSED_BIN_FILE"
echo "Labels:         $LABEL_FILE"
echo "---------------------------------------"

python3 "${UTILS_DIR}/visualize_point_cloud.py" \
    "$RAW_BIN_FILE" \
    "$PROCESSED_BIN_FILE" \
    "$LABEL_FILE" \
    "$OUTPUT_FILE"

# Check the exit code of the python script
if [ $? -eq 0 ]; then
    echo "✅ Success! Output saved to ${OUTPUT_FILE}"
else
    echo "❌ An error occurred during visualization."
fi
