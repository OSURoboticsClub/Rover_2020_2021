cmake_minimum_required(VERSION 2.8.3)
project(rover_arm_control)

## Add flag for c++11
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  joint_limits_interface
  urdf
  controller_manager
  actionlib
  control_msgs
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  tf
  transmission_interface
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES arm_hw_interface
  CATKIN_DEPENDS hardware_interface joint_limits_interface urdf controller_manager actionlib control_msgs roscpp sensor_msgs std_msgs trajectory_msgs tf transmission_interface
#  DEPENDS arm_hw_interface
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library

# Hardware Interface
add_library(arm_hw_interface arm_hw_interface/arm_hw_interface.cpp)
target_link_libraries(arm_hw_interface
  ${catkin_LIBRARIES}
  simplemotion2
)

## Specifying sources
set(${PROJECT_NAME}_SOURCES
  arm_hw_interface/arm_state.cpp
  arm_hw_interface/arm_hw_interface.cpp
  arm_hw_interface/arm_hw_interface_node.cpp)


## Declare a C++ executable
add_executable(arm_hw_interface_node ${${PROJECT_NAME}_SOURCES})

## Add dependencies
add_dependencies(arm_hw_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(arm_hw_interface_node
  arm_hw_interface
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(DIRECTORY config/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

## Mark executables and/or libraries for installation
install(TARGETS arm_hw_interface arm_hw_interface_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)