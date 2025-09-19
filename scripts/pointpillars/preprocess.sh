#!/bin/bash

CALL_DIR="$(pwd)"

HOME_DIR="/workspace"
DATASET_DIR="$HOME_DIR/datasets"
PROCESSED_DIR="$HOME_DIR/processed_data"
POINT_PILLAR_DIR="$HOME_DIR/pointpillars"

echo "Starting Point Pillar preprocessing..."

# ==============================================================================
# Check if we are preprocessing processed data
# ==============================================================================

USE_PROCESSED=0

if [[ $# -gt 1 || ($# -eq 1 && "$1" != "--use-processed") ]]; then
    echo "Error: Invalid arguments." >&2
    echo "Usage: $0 [--use-processed]" >&2
    exit 1
fi

if [[ "$1" == "--use-processed" ]]; then
    echo "Preprocessing with processed data selected..."
    USE_PROCESSED=1
fi

# ==============================================================================
# Cleaning up the dataset folder
# ==============================================================================

echo "Cleaning up dataset directory..."
find "$DATASET_DIR" -mindepth 1 -maxdepth 1 ! -name 'training' ! -name 'testing' -exec rm -rf {} +
rm -rf "$DATASET_DIR/training/velodyne_reduced"
rm -rf "$DATASET_DIR/testing/velodyne_reduced"

# ==============================================================================
# Main logic for --use-processed flag
# ==============================================================================

if [ $USE_PROCESSED -eq 1 ]; then
    echo "Using preprocessed data for temporary processing..."

    echo "Cleaning up dataset directory..."
    find "$PROCESSED_DIR" -mindepth 1 -maxdepth 1 ! -name 'training' ! -name 'testing' -exec rm -rf {} +
    rm -rf "$PROCESSED_DIR/training/velodyne_reduced"
    rm -rf "$PROCESSED_DIR/testing/velodyne_reduced"

    # Splits to process
    SPLITS=("training" "testing")

    for SPLIT in "${SPLITS[@]}"; do
        DATASET_SPLIT_DIR="$DATASET_DIR/$SPLIT"
        PROCESSED_SPLIT_DIR="$PROCESSED_DIR/$SPLIT"
        VEL_DIR="$DATASET_SPLIT_DIR/velodyne"
        PRO_VEL_DIR="$PROCESSED_DIR/$SPLIT/velodyne"
        
        # 1. mv velodyne to velodyne.orig
        if [ -d "$VEL_DIR" ] && ! [ -L "$VEL_DIR" ]; then
            echo "Backing up $VEL_DIR to $VEL_DIR.orig..."
            mv "$VEL_DIR" "$VEL_DIR.orig"
        fi

        # 2. soft link velodyne/ to the corresponding processed_data/split/velodyne
        echo "Linking $PRO_VEL_DIR to $VEL_DIR..."
        ln -s "$PRO_VEL_DIR" "$VEL_DIR"
    done

    # 3. Run the PointPillars preprocessing script
    echo "Running Point Pillars preprocessing with linked data..."
    python "$POINT_PILLAR_DIR/pre_process_kitti.py" --data_root "$DATASET_DIR" || true

    # 4. mv any new generated content into processed_data/ with the respective splits
    echo "Moving any files or directories outside of training/ and testing/ to processed_data/..."
    find "$DATASET_DIR" -mindepth 1 -maxdepth 1 ! -name 'training' ! -name 'testing' -exec mv {} "$PROCESSED_DIR" \;

    for SPLIT in "${SPLITS[@]}"; do
        NEW_VEL_REDUCED="$DATASET_DIR/$SPLIT/velodyne_reduced"
        PROCESSED_SPLIT_DIR="$PROCESSED_DIR/$SPLIT"
        if [ -d "$NEW_VEL_REDUCED" ]; then
            mv "$NEW_VEL_REDUCED" "$PROCESSED_SPLIT_DIR"
        fi
    done

    # 5. Revert dataset/ to how it was before
    echo "Restoring original dataset structure..."
    for SPLIT in "${SPLITS[@]}"; do
        DATASET_SPLIT_DIR="$DATASET_DIR/$SPLIT"
        VEL_DIR="$DATASET_SPLIT_DIR/velodyne"

        if [ -L "$VEL_DIR" ]; then
            echo "Removing symlink $VEL_DIR..."
            rm "$VEL_DIR"
        fi
        
        if [ -d "$VEL_DIR.orig" ]; then
            echo "Restoring $VEL_DIR from backup..."
            mv "$VEL_DIR.orig" "$VEL_DIR"
        fi
    done
fi

# 6. Call preprocessing on dataset/
# This is the final preprocessing step on the original data as requested
echo "Running final Point Pillars preprocessing on the original dataset..."
python "$POINT_PILLAR_DIR/pre_process_kitti.py" --data_root "$DATASET_DIR" || true

echo "Point Pillar preprocessing complete."
cd "$CALL_DIR"