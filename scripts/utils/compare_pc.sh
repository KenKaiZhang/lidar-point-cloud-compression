#!/bin/bash

# ==============================================================================
# Script Configuration
# ==============================================================================

set -e

HOME_DIR="/workspace"
RAW_DATASET_DIR="$HOME_DIR/dataset"
PRO_DATASET_DIR="$HOME_DIR/processed_datasets/rcpcc"

PYTHON_SCRIPT="$HOME_DIR/utils/compare_pc.py"

# ==============================================================================
# Input Validation
# ==============================================================================

if [ -z "$1" ]; then
    echo "Usage: ./compare_pc.sh <file_id>"
    echo "Example: ./compare_pc.sh 000008"
    exit 1
fi

FILE_ID=$(printf "%06d" "$1")

echo "Preparing to compare files for ID: $FILE_ID"

# ==============================================================================
# File Path Definitions
# ==============================================================================
RAW_BIN_FILE="$RAW_DATASET_DIR/training/velodyne/$FILE_ID.bin"
PRO_BIN_FILE="$PRO_DATASET_DIR/training/velodyne/$FILE_ID.bin"
LABEL_FILE="$RAW_DATASET_DIR/training/label_2/$FILE_ID.txt"
CALIB_FILE="$RAW_DATASET_DIR/training/calib/$FILE_ID.txt"
OUTPUT_FILE="comparison_${FILE_ID}.html"

for f in "$PYTHON_SCRIPT" "$RAW_BIN_FILE" "$PRO_BIN_FILE" "$LABEL_FILE" "$CALIB_FILE"; do
    if [ ! -f "$f" ]; then
        echo "Error: Required file not found at '$f'"
        exit 1
    fi
done

# ==============================================================================
# Execute Comparison
# ==============================================================================

echo "All files found. Running the comparison script..."
echo "  - Raw Cloud: $RAW_BIN_FILE"
echo "  - Processed Cloud: $PRO_BIN_FILE"
echo "  - Labels: $LABEL_FILE"
echo "  - Calibration: $CALIB_FILE"
echo "  - Output: $OUTPUT_FILE"

python3 "$PYTHON_SCRIPT" \
    "$RAW_BIN_FILE" \
    "$PRO_BIN_FILE" \
    "$LABEL_FILE" \
    "$CALIB_FILE" \
    "$OUTPUT_FILE"

echo "Comparison complete. View the results by opening '$OUTPUT_FILE' in your browser."