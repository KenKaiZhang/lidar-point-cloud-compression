#!/bin/bash

CALL_DIR="$(pwd)"
HOME_DIR="/workspace"

RCPCC_DIR="$HOME_DIR/rcpcc"

echo "Starting build for RCPCC code..."

mkdir -p "$RCPCC_DIR/build"
cd "$RCPCC_DIR/build"
cmake ../src && make

echo "RCPCC code build complete."

cd "$CALL_DIR"