# Tested on:

1. Ubuntu 12.04 x64 Virtual Machine via VMware Fusion 6.0.2
2. ROS Hydro
3. CloudSim Pre-Release 2.0.0 (simulator-stable)

# Quick Deploy:

## Clone repo in home folder
hg clone http://bitbucket.org/ammpedro/cloudsim-ed-actuation

## Run deploy.bash

. ~/cloudsim-ed-actuation/deploy.bash

## Workspace Location
~/code/ws/src/cloudsim-ed-actuation

## Launch a task
roslaunch cloudsim_ed_actuation cloudsim_ed_actuation_challenge_01.launch


#How to install on a CloudSim instance

## create catkin workspace
cd /home/ubuntu
mkdir actu
mkdir actu/src
cd actu/src

## get the code
hg clone http://bitbucket.org/ammpedro/cloudsim-ed-actuation

## source ROS
. /opt/ros/groovy/setup.bash
cd /home/ubuntu/actu

## compile
catkin_make install

## source files and publish models to gzweb
. /home/ubuntu/actu/src/cloudsim-ed-actuation/setup.bash
cd /home/ubuntu/cloudsim/gzweb

## webify every model on the local machine
./deploy.sh -m local

## how to start it:
roslaunch cloudsim_ed_actuation cloudsim_ed_actuation_challenge_01.launch
