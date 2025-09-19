#!/bin/bash

CALL_DIR="$(pwd)"
HOME_DIR="/workspace"

GT_DATASET_DIR="$HOME_DIR/datasets"
POINT_PILLAR_DIR="$HOME_DIR/pointpillars"

echo "Starting Point Pillar evaluation..."

# ==============================================================================
# Check if using processed training/val data for evaluation
# ==============================================================================
if [[ $# -gt 1 || ($# -eq 1 && "$1" != "--use-processed") ]]; then
    echo "Error: Invalid arguments." >&2
    echo "Usage: $0 [--use-processed]" >&2
    exit 1
fi

if [[ "$1" == "--use-processed" ]]; then
    DATASET_DIR="$HOME_DIR/processed_data"
else
    DATASET_DIR="$GT_DATASET_DIR"
fi

echo "Evaluating with $DATASET_DIR..."

# ==============================================================================
# Run the PointPillars evaluation script
# ==============================================================================
python "$POINT_PILLAR_DIR/evaluate.py" --ckpt "$POINT_PILLAR_DIR/pretrained/epoch_160.pth" --data_root "$DATASET_DIR" --gt_data_root "$GT_DATASET_DIR"
echo
echo "Point Pillar evaluation complete."

cd "$CALL_DIR"