#!/bin/bash
set -e

THIS_DIR="$(readlink -m "$(dirname "$0")")"
ROS_DIR="$THIS_DIR/ros"

# Check that the build folder exists
if [ ! -d "$ROS_DIR/build" ]
then
    echo "Please run the build.sh command first!"
    exit 1
fi

# Launch simulator if it's not running yet
if pgrep "system_integr" > /dev/null
then
    echo "Great, the simulator is already running!"
else
    echo "Starting the simulator..."
    $THIS_DIR/ros/src/styx/unity_simulator_launcher.sh &
fi

# Launch ROS nodes
docker run --rm=true --tty=true --interactive=true               \
           --user=$(id --user):$(id --group)                     \
           --volume="/tmp":"/.ros"                               \
           --volume="$THIS_DIR":"$THIS_DIR"                      \
           --workdir="$ROS_DIR"                                  \
           --network=host                                        \
           carlosgalvezp/carnd_capstone /bin/bash -c             \
           "source /opt/ros/kinetic/setup.bash;
            source devel/setup.bash;
            roslaunch launch/styx.launch"

