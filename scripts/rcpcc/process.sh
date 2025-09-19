#!/bin/bash

CALL_DIR="$(pwd)"
HOME_DIR="/workspace"

DATASET_DIR="$HOME_DIR/datasets"
PRO_DATASET_DIR="$HOME_DIR/processed_data/rcpcc"
RCPCC_DIR="$HOME_DIR/rcpcc"

echo "Starting RCPCC processing..."

# ==============================================================================
# Processing training data
# ==============================================================================

TRAINING_DIR="$DATASET_DIR/training/velodyne"
TRAINING_OUT_DIR="$PRO_DATASET_DIR/training/velodyne"

echo "Processing data from $TRAINING_DIR to $TRAINING_OUT_DIR..."
"$RCPCC_DIR/build/process" "$TRAINING_DIR" 0 -o "$TRAINING_OUT_DIR" > /dev/null 2>&1

# ==============================================================================
# Processing testing data
# ==============================================================================

TESTING_DIR="$DATASET_DIR/testing/velodyne"
TESTING_OUT_DIR="$PRO_DATASET_DIR/testing/velodyne"

echo "Processing data from $TESTING_DIR to $TESTING_OUT_DIR..."
"$RCPCC_DIR/build/process" "$TESTING_DIR" 0 -o "$TESTING_OUT_DIR" > /dev/null 2>&1

echo "RCPCC processing complete."

cd "$CALL_DIR"