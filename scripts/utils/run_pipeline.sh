#!/bin/bash

# =================================================================
# Data Processing and Evaluation Pipeline for RCPCC and PointPillars
# =================================================================

# Define environment variables based on the user's context
HOME_DIR="/workspace"
SCRIPTS_DIR="$HOME_DIR/scripts"

# Set -e: Exit immediately if a command exits with a non-zero status.
# This ensures that if 'build.sh' fails, the rest of the pipeline is not executed.
set -e

echo "Starting data processing pipeline..."
echo "Scripts directory: $SCRIPTS_DIR"

# 1. Run the RCPCC processing script
echo
echo "--- Step 1/3: Running RCPCC process script..."
"$SCRIPTS_DIR/rcpcc/process.sh"

echo "RCPCC process script finished successfully."

# 2. Run the PointPillars preprocessing script for RCPCC data
echo
echo "--- Step 2/3: Running PointPillars preprocessing..."
"$SCRIPTS_DIR/pointpillars/preprocess.sh" -p rcpcc

echo "PointPillars preprocessing finished successfully."

# 3. Run the PointPillars evaluation script for RCPCC data
echo
echo "--- Step 3/3: Running PointPillars evaluation..."
"$SCRIPTS_DIR/pointpillars/evaluate.sh" -p rcpcc

echo "================================================================="