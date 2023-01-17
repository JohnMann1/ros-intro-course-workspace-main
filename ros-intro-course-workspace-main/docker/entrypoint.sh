#!/bin/bash
# Basic entrypoint for ROS / Catkin Docker containers

# Source ROS and Catkin workspaces
source ~/.bashrc

cd ~/catkin_ws/src

if [[ ! -d ~/catkin_ws/src/turtlebot3_simulations ]]
then
    git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
fi

cd ~/catkin_ws \

source ~/.bashrc

source /opt/ros/noetic/setup.bash
catkin_make

source ~/.bashrc
source ~/catkin_ws/devel/setup.bash

# Execute the command passed into this entrypoint
exec "$@"