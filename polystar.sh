#!/bin/bash

# Setup env

export WS="/home/polystar/robomaster-2023-cv/ros_ws"

source /home/polystar/.bashrc
source $WS/devel/setup.bash

# Run launch file

sleep 10 # safeguard
cd $WS
roslaunch robomaster.launch

