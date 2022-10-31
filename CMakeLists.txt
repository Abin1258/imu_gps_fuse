cmake_minimum_required(VERSION 3.0.2)
project(imu_gps_fuse)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geodesy
  geographic_msgs
  geometry_msgs
  nav_msgs
  roscpp
  roslib
  sensor_msgs
)


## catkin specific configuration ##
catkin_package(
 INCLUDE_DIRS
 include
)
## Specify additional locations of header files
## Your package locations should be listed before other locations

add_subdirectory(imu_gps_localizer)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  imu_gps_localizer/include
  include
)

add_library(imu_gps_fuse_lib
	src/imu_gps_fuse.cpp
)
target_link_libraries(imu_gps_fuse_lib
  ${catkin_LIBRARIES}
  imu_gps_localizer_lib
  glog
)
## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/imu_gps_fuse.cpp
# )

## Specify libraries to link a library or executable target against
add_executable(imu_gps_fuse src/imu_gps_fuse.cpp)
target_link_libraries(imu_gps_fuse
 imu_gps_fuse_lib
)