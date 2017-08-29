#!/bin/bash
set -e

THIS_DIR="$(readlink -m "$(dirname "$0")")"
docker build -f "$THIS_DIR/Dockerfile" "$THIS_DIR/.."
