#!/bin/bash
set -e

THIS_DIR="$(readlink -m "$(dirname "$0")")"
ROS_DIR="$THIS_DIR/ros"

# Re-build the code
echo "Building the code..."
$THIS_DIR/build.sh

# Launch simulator if it's not running yet
if pgrep "system_integr" > /dev/null
then
    echo "Great, the simulator is already running!"
else
    echo "Starting the simulator..."
    $THIS_DIR/ros/src/styx/unity_simulator_launcher.sh
fi

# Launch ROS nodes
docker run --rm=true --tty=true --interactive=true               \
           --user=$(id --user):$(id --group)                     \
           --volume="/tmp":"/.ros"                               \
           --volume="$THIS_DIR":"$THIS_DIR"                      \
           --workdir="$ROS_DIR"                                  \
           --network=host                                        \
           --env DISPLAY                                         \
           --volume /tmp/.X11-unix:/tmp/.X11-unix                \
           eurobots/carnd_capstone /bin/bash -c                  \
           "source /opt/ros/kinetic/setup.bash;
            source devel/setup.bash;
            roslaunch launch/styx.launch"

