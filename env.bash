#!/bin/bash

#source /opt/ros/kinetic/setup.bash
#export ROS_IP=192.168.100.235
#export ROS_MASTER_URI=http://192.168.100.3:11311/
export ROS_IP=10.8.0.7
export ROS_MASTER_URI=http://10.8.0.2:11311/

source /home/dmitrydzz/dev/mitya3/ros3/devel/setup.bash

exec "$@"
