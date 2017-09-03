#!/bin/bash
set -e

THIS_DIR="$(pwd)"
ROS_DIR="$THIS_DIR/ros"

# Re-build the code
echo "Building the code..."
$THIS_DIR/build.sh

# Launch simulator if it's not running yet
if pgrep "mac_sys_int" > /dev/null
then
    echo "Great, the simulator is already running!"
else
    echo "Starting the simulator..."
    cd $THIS_DIR/ros/src/styx
    ./unity_simulator_launcher.sh &
    cd $THIS_DIR
fi

# Launch ROS nodes
docker run --rm=true --tty=true --interactive=true               \
           --volume="/tmp":"/.ros"                               \
           --volume="$THIS_DIR":"$THIS_DIR"                      \
           --workdir="$ROS_DIR"                                  \
            -p 4567:4567 \
           carlosgalvezp/carnd_capstone /bin/bash -c             \
           "source /opt/ros/kinetic/setup.bash;
            source devel/setup.bash;
            roslaunch launch/styx.launch"

