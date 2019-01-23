cmake_minimum_required(VERSION 2.8.3)
project(uchile_maps)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_ros
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  uchile_msgs
  uchile_srvs
  uchile_util
)

find_package(PCL 1.7 REQUIRED COMPONENTS common)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES uchile_nav
  CATKIN_DEPENDS 
    geometry_msgs
    pcl_ros
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
    std_srvs
    tf
    uchile_msgs
    uchile_srvs
    uchile_util

  DEPENDS pcl
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Map Analyzer
add_executable(map_analyzer src/MapAnalyzer.cpp)
add_dependencies(map_analyzer ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_analyzer ${catkin_LIBRARIES})
