#!/bin/bash

echo "Creating directories..."
. /opt/ros/groovy/setup.bash
mkdir /home/ubuntu/code
mkdir ws
mkdir ws/src
cd /home/ubuntu/code/ws/src

echo "Cloning repo..."
hg clone https://bitbucket.org/ammpedro/cloudsim-ed-actuation
echo "Initializing catkin workspace"
catkin_init_workspace
cd ..

echo "Building catkin workspace"
catkin_make install
source devel/setup.bash
. install/setup.bash
. install/share/cloudsim-ed-actuation/setup.sh

echo "Setup Notebooks"
cp /src/cloudsim-ed-actuation/notebooks/. /home/ubuntu/cloudsim/notebook/ -R
