#!/bin/bash
set -e

THIS_DIR="$(pwd)"
ROS_DIR="$THIS_DIR/ros"

CONTAINER_ID="$(docker ps | grep carlosgalvezp/carnd_capstone | cut -b 1-12)"

FILENAME="rosout.log"

docker exec -it $CONTAINER_ID  \
           bash -c \
           "source /opt/ros/kinetic/setup.bash
           roscd log
           echo tail -f $FILENAME
           tail -f -n 50 $FILENAME"

#dbw_node-3-stdout.log
#dbw_node-3.log
#master.log
#pure_pursuit-5-stdout.log
#roslaunch-f95cb874a253-1.log
#rosout-1-stdout.log
#rosout.log
#styx_server-2-stdout.log
#styx_server-2.log
#tl_detector-7.log
#waypoint_loader-4-stdout.log
#waypoint_loader-4.log
#waypoint_updater-6-stdout.log
#waypoint_updater-6.log
