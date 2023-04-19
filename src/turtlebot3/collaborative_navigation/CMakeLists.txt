cmake_minimum_required(VERSION 3.0.2)
project(collaborative_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  geometry_msgs
  move_base
  nav_core
  nav_msgs
  base_local_planner
  pluginlib
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    tf
    geometry_msgs
    # move_base
    nav_core
    nav_msgs
    base_local_planner
    pluginlib
)

# LIBRARY
add_library(global_planner_lib
  src/tf_listener.cpp
  src/path_generator.cpp
  src/path_visualization.cpp
)
target_include_directories(global_planner_lib PUBLIC 
  include/collaborative_navigation
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(global_planner_lib 
  ${catkin_LIBRARIES}
)
# EXECUTE
add_executable(system_global_planner_node
  src/system_global_planner.cpp
)
target_include_directories(system_global_planner_node PUBLIC 
  include/collaborative_navigation
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(system_global_planner_node 
  global_planner_lib
  ${catkin_LIBRARIES}
)


