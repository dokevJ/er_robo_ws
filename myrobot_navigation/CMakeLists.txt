cmake_minimum_required(VERSION 3.0.2)
project(myrobot_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_core
  nav_msgs
  roscpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  base_local_planner
  costmap_2d
  cmake_modules
  pluginlib
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES myrobot_navigation
 CATKIN_DEPENDS 
  nav_core 
  nav_msgs 
  roscpp 
  sensor_msgs 
  tf2 
  tf2_ros
  tf2_geometry_msgs
  base_local_planner
  costmap_2d
  cmake_modules
  pluginlib  
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)


find_package(Eigen3 REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})
