#!/bin/bash

. /opt/ros/groovy/setup.bash

#delete all first

echo "Creating directories..."
mkdir ~/code
cd ~/code
mkdir ws
mkdir ws/src
cd ws/src

echo "Transfer repo..."
cp -r ~/cloudsim-ed-actuation /home/ubuntu/code/ws/src
echo "Initializing catkin workspace"
catkin_init_workspace
cd ..

echo "Building catkin workspace"
catkin_make install
. devel/setup.bash
. install/setup.bash
. install/share/cloudsim_ed_actuation/setup.sh

# /home/ubuntu/cloudsim/notebook/
#echo "Setup Notebooks"
#cp src/cloudsim-ed-actuation/notebooks/. /home/ubuntu/cloudsim/notebook/ -R

echo "Delete repo files"
rm -r ~/cloudsim-ed-actuation
