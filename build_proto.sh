#!/bin/bash
echo "Current directory: $(pwd)"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTO_DIR="$SCRIPT_DIR/proto-public-api"
PYTHON_OUT_DIR="$SCRIPT_DIR/hex_arm"
echo "Proto directory: $PROTO_DIR"
echo "Python output directory: $PYTHON_OUT_DIR"
protoc -I="$PROTO_DIR" --python_out="$PYTHON_OUT_DIR" "$PROTO_DIR"/*.proto