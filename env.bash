#!/bin/bash

#source /opt/ros/kinetic/setup.bash
export ROS_IP=192.168.100.235
export ROS_MASTER_URI=http://192.168.100.3:11311/

source /home/dmitrydzz/dev/mitya3/ros3/devel/setup.bash

exec "$@"
