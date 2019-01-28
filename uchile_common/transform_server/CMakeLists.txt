cmake_minimum_required(VERSION 2.8.3)
project(transform_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  kdl_parser
  roscpp
  sensor_msgs
  std_msgs
  std_srvs
  tf
  tf_conversions
  uchile_msgs
  uchile_srvs
)
find_package(orocos_kdl REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen REQUIRED)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    kdl_parser
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf
    tf_conversions
    uchile_msgs
    uchile_srvs
  DEPENDS orocos_kdl
)

###########
## Build ##
###########

include_directories(SYSTEM ${EIGEN_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)
link_directories(${orocos_kdl_LIBRARY_DIRS})

# Pose transformer
add_executable(simple_pose_transformer src/TransformPoseSimple.cpp)
add_dependencies(simple_pose_transformer ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_pose_transformer ${catkin_LIBRARIES})