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
)

## System dependencies are found with CMake's conventions
find_package(VISP REQUIRED)

find_package(OpenCV REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp roslib
DEPENDS VISP OpenCV
)

###########
## Build ##
###########

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} src/tps_robot.cpp include/tps_robot/tps_robot.h)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${VISP_LIBRARIES})

add_executable(simu_4points src/simu_4points.cpp)
target_link_libraries(simu_4points ${PROJECT_NAME}  ${OpenCV_LIBS})

#add_executable(control src/control.cpp src/modeles.cpp)
#target_link_libraries(control ${PROJECT_NAME})

