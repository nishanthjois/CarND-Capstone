#!/bin/bash
set -e

THIS_DIR="$(readlink -m "$(dirname "$0")")"
docker run --rm=true --tty=true --volume="$THIS_DIR":"$THIS_DIR" \
           --workdir="$THIS_DIR" --user=$(id -u):$(id -g)        \
           carlosgalvezp/carnd_capstone /bin/bash -c             \
           "source /opt/ros/kinetic/setup.bash; catkin_make"
