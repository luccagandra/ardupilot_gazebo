#!/bin/bash
set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Starting install"

# get the current commit SHA
SHA=`git rev-parse HEAD`

# get the current package name
PACKAGE_NAME=${PWD##*/}

echo "clone uav_ros_simulation"
cd
git clone https://github.com/lmark1/uav_ros_simulation.git
cd uav_ros_simulation

echo "running the main install.sh"
./installation/install_and_setup_workspace.sh

# checkout the SHA
cd ~/uav_ros_simulation/.gitman/$PACKAGE_NAME
git checkout "$SHA"

source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "install ended"