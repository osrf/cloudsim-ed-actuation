#!/bin/bash

. /opt/ros/groovy/setup.bash

echo "Creating directories..."
mkdir ~/code
mkdir ~/cloudsim/notebook
cd ~/code
mkdir ws
mkdir ws/src
cd ws/src

echo "Transfer repo..."
cp -r ~/cloudsim-ed-actuation ~/code/ws/src
echo "Initializing catkin workspace"
catkin_init_workspace
cd ..

echo "Building catkin workspace"
catkin_make install
. devel/setup.bash
. install/share/cloudsim_ed_actuation/setupx.bash

echo "Setup Notebooks"
cp src/cloudsim-ed-actuation/notebooks/. ~/cloudsim/notebook/ -R

echo "Delete repo files"
rm -r ~/cloudsim-ed-actuation
