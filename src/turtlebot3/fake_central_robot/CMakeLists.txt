cmake_minimum_required(VERSION 3.0.2)
project(fake_central_robot)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
  fake_central_odom_node 
  src/fake_central_odom.cpp
)

target_include_directories(fake_central_odom_node PUBLIC
  include/fake_central_robot
)
