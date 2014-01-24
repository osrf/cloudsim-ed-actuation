
# not needed since we moved plugin target directory into ${CMAKE_INSTALL_PREFIX}/lib/...
# export LD_LIBRARY_PATH=`pwd`/install/lib/scoring_plugins/plugins:$LD_LIBRARY_PATH
# Make kobuki_description and turtlebot_description gazebo models visible to gazebo at runtime
if [ -f /usr/share/gazebo/setup.sh ]; then
  . /usr/share/gazebo/setup.sh
elif [ -f /usr/share/gazebo/setup.sh ]; then
  . /usr/share/gazebo/setup.sh
else
  echo "Warning: failed to find Gazebo's setup.sh.  You will need to source it manually."
fi

export GAZEBO_MODEL_PATH=`rospack find basic_robot`:`rospack find test_motor`:$GAZEBO_MODEL_PATH
