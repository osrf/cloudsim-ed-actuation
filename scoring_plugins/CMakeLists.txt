cmake_minimum_required(VERSION 2.8.3)
project(scoring_plugins)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")


find_package(catkin REQUIRED COMPONENTS
             roscpp
             std_msgs
             gazebo_plugins
             message_generation
)

add_message_files(
  DIRECTORY msg
  FILES
  Score.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES drcsim_gazebo_ros_plugins
  CATKIN_DEPENDS gazebo_plugins
  message_runtime
  message_generation
#  DEPENDS system_lib
)

find_package(gazebo REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a cpp library


add_library(GateScoringPlugin src/GateScoringPlugin.cc)
target_link_libraries(GateScoringPlugin ${catkin_LIBRARIES})
add_dependencies(GateScoringPlugin scoring_plugins_generate_messages_cpp ${catkin_EXPORTED_TARGETS})


### executables, utilities for debugging atlas simulation, should not depend on gazebo


#############
## Install ##
#############
install(TARGETS GateScoringPlugin 
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
