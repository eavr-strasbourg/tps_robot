cmake_minimum_required(VERSION 2.8.3)
project(tps_robot)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tf
  sensor_msgs
  cv_bridge
  urdf
  visp
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp roslib visp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
# TODO: Check names of system library include directories (visp)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} src/tps_robot.cpp include/tps_robot/tps_robot.h)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(simu4points src/simu4points.cpp)
target_link_libraries(simu4points ${PROJECT_NAME})

add_executable(control src/control.cpp)
target_link_libraries(control ${PROJECT_NAME})
