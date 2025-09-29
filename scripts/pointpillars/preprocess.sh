#!/bin/bash

# Configuration Variables
CALL_DIR="$(pwd)"
HOME_DIR="/workspace"
DATASET_DIR="$HOME_DIR/dataset"
PROCESSED_DIR="$HOME_DIR/processed_datasets"
POINT_PILLAR_DIR="$HOME_DIR/pointpillars"
SPLITS=("training" "testing")

echo "Starting Point Pillar preprocessing..."

# ==============================================================================
# Argument Parsing 
# ==============================================================================

PROCESSED_SUBDIR=""
USE_PROCESSED=0
while [[ "$#" -gt 0 ]]; do
    case "$1" in
        -p|--processed)
            if [ -n "$2" ] && ! [[ "$2" =~ ^- ]]; then
                PROCESSED_SUBDIR="$2"
                USE_PROCESSED=1
                shift # past argument
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
    shift # past argument or value
done

# ==============================================================================
# Cleaning up the dataset folder (Always runs)
# This clears out previous preprocessing artifacts from the original dataset dir
# ==============================================================================

echo "Cleaning up temporary artifacts from the dataset directory..."
# Remove non-training/testing directories at the root level of DATASET_DIR
find "$DATASET_DIR" -mindepth 1 -maxdepth 1 ! -name 'training' ! -name 'testing' -exec rm -rf {} +
# Remove velodyne_reduced from splits
rm -rf "$DATASET_DIR/training/velodyne_reduced"
rm -rf "$DATASET_DIR/testing/velodyne_reduced"


# ==============================================================================
# Main logic for -p <folder> flag (Using processed velodyne data for info file generation)
# ==============================================================================

if [ $USE_PROCESSED -eq 1 ]; then
    BASE_PROCESSED_PATH="$PROCESSED_DIR/$PROCESSED_SUBDIR"
    echo "Using preprocessed data from '$PROCESSED_SUBDIR' for temporary processing..."

    # Ensure the target directory for final processed artifacts exists
    mkdir -p "$BASE_PROCESSED_PATH/training"
    mkdir -p "$BASE_PROCESSED_PATH/testing"

    echo "Cleaning up old preprocessing artifacts in '$PROCESSED_SUBDIR'..."
    # Remove old root-level info files (like kitti_infos_train.pkl)
    find "$BASE_PROCESSED_PATH" -maxdepth 1 -type f -exec rm -f {} +
    # Remove old velodyne_reduced directories
    rm -rf "$BASE_PROCESSED_PATH/training/velodyne_reduced"
    rm -rf "$BASE_PROCESSED_PATH/testing/velodyne_reduced"

    for SPLIT in "${SPLITS[@]}"; do
        DATASET_SPLIT_DIR="$DATASET_DIR/$SPLIT"
        PROCESSED_SPLIT_DIR="$BASE_PROCESSED_PATH/$SPLIT"
        VEL_DIR="$DATASET_SPLIT_DIR/velodyne"
        PRO_VEL_DIR="$PROCESSED_SPLIT_DIR/velodyne"

        # 1. Back up original velodyne/ to velodyne.orig
        if [ -d "$VEL_DIR" ] && ! [ -L "$VEL_DIR" ]; then
            echo "Backing up $VEL_DIR to $VEL_DIR.orig..."
            mv "$VEL_DIR" "$VEL_DIR.orig"
        fi

        # 2. Soft link dataset/split/velodyne/ to the corresponding processed_data/sub_folder/split/velodyne/
        if [ -d "$PRO_VEL_DIR" ]; then
            echo "Linking processed $PRO_VEL_DIR to $VEL_DIR..."
            ln -s "$PRO_VEL_DIR" "$VEL_DIR"
        else
            echo "Warning: Source processed directory not found: $PRO_VEL_DIR. Skipping link."
        fi
    done

    # 3. Run the PointPillars preprocessing script (This creates info files and velodyne_reduced in DATASET_DIR)
    echo "Running Point Pillars preprocessing with linked data..."
    python "$POINT_PILLAR_DIR/pre_process_kitti.py" --data_root "$DATASET_DIR" || true

    # 4. Move new generated content from DATASET_DIR into the specified PROCESSED_SUBDIR/
    echo "Moving newly generated artifacts (info files, velodyne_reduced) to $BASE_PROCESSED_PATH/..."
    # Move root level files
    find "$DATASET_DIR" -mindepth 1 -maxdepth 1 ! -name 'training' ! -name 'testing' -exec mv {} "$BASE_PROCESSED_PATH" \;

    # Move velodyne_reduced directories
    for SPLIT in "${SPLITS[@]}"; do
        NEW_VEL_REDUCED="$DATASET_DIR/$SPLIT/velodyne_reduced"
        PROCESSED_SPLIT_DIR="$BASE_PROCESSED_PATH/$SPLIT"
        if [ -d "$NEW_VEL_REDUCED" ]; then
            echo "Moving $SPLIT/velodyne_reduced to $PROCESSED_SPLIT_DIR/"
            mv "$NEW_VEL_REDUCED" "$PROCESSED_SPLIT_DIR"
        fi
    done

    # 5. Revert dataset/ to how it was before
    echo "Restoring original dataset structure..."
    for SPLIT in "${SPLITS[@]}"; do
        DATASET_SPLIT_DIR="$DATASET_DIR/$SPLIT"
        VEL_DIR="$DATASET_SPLIT_DIR/velodyne"

        if [ -L "$VEL_DIR" ]; then
            echo "Removing temporary symlink $VEL_DIR..."
            rm "$VEL_DIR"
        fi

        if [ -d "$VEL_DIR.orig" ]; then
            echo "Restoring $VEL_DIR from backup..."
            mv "$VEL_DIR.orig" "$VEL_DIR"
        fi
    done
fi

# 6. Call preprocessing on dataset/ (Always runs)
# This is the final preprocessing step on the original data as requested
echo "Running final Point Pillars preprocessing on the original dataset..."
python "$POINT_PILLAR_DIR/pre_process_kitti.py" --data_root "$DATASET_DIR" || true

echo "Point Pillar preprocessing complete."
cd "$CALL_DIR"
