#!/bin/bash

CALL_DIR="$(pwd)"
HOME_DIR="/workspace"

GT_DATASET_DIR="$HOME_DIR/dataset"
PROCESSED_DIR="$HOME_DIR/processed_datasets"
POINT_PILLAR_DIR="$HOME_DIR/pointpillars"

echo "Starting Point Pillar evaluation..."

# ==============================================================================
# Argument Parsing (Implementing -p <folder> logic)
# ==============================================================================

PROCESSED_SUBDIR=""
DATASET_DIR="$GT_DATASET_DIR"

while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -p|--processed)
            if [ -n "$2" ] && ! [[ "$2" =~ ^- ]]; then
                PROCESSED_SUBDIR="$2"
                DATASET_DIR="$PROCESSED_DIR/$PROCESSED_SUBDIR"
                shift
            else
                echo "Error: Argument for $1 is missing or invalid." >&2
                echo "Usage: $0 [-p <folder>]" >&2
                exit 1
            fi
            ;;
        *)
            echo "Error: Unknown argument $1." >&2
            echo "Usage: $0 [-p <folder>]" >&2
            exit 1
            ;;
    esac
    shift
done

echo "Evaluating with $DATASET_DIR..."

# ==============================================================================
# Run the PointPillars evaluation script
# ==============================================================================
python "$POINT_PILLAR_DIR/evaluate.py" --ckpt "$POINT_PILLAR_DIR/pretrained/epoch_160.pth" --data_root "$DATASET_DIR" --gt_data_root "$GT_DATASET_DIR"
echo
echo "Point Pillar evaluation complete."

cd "$CALL_DIR"
