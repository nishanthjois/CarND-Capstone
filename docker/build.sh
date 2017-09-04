#!/bin/bash
set -e

THIS_DIR="$(readlink -m "$(dirname "$0")")"
docker build --file "$THIS_DIR/Dockerfile"             \
             --tag eurobots/carnd_capstone:latest      \
             "$THIS_DIR/.."
