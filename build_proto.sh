#!/bin/bash
echo "Current directory: $(pwd)"
PROTO_DIR="$(pwd)/proto-public-api"
PYTHON_OUT_DIR="$(pwd)/hex_arm"
echo "Proto directory: $PROTO_DIR"
echo "Python output directory: $PYTHON_OUT_DIR"
protoc -I="$PROTO_DIR" --python_out="$PYTHON_OUT_DIR" "$PROTO_DIR"/*.proto