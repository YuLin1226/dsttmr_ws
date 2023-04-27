cmake_minimum_required(VERSION 3.0.2)
project(fake_central_robot_navigation)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
)


catkin_package(
  INCLUDE_DIRS
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    tf
    geometry_msgs
)

# include_directories(
# # include
#   ${catkin_INCLUDE_DIRS}
# )

add_library(tf_listener
  src/tf_listener.cpp
)
target_include_directories(tf_listener PUBLIC
  include/fake_central_robot_navigation
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
  tf_listener
  ${catkin_LIBRARIES}
)

add_library(single_robot_cmd_vel_calculator
  src/single_robot_cmd_vel_calculator.cpp
)
target_include_directories(single_robot_cmd_vel_calculator PUBLIC
  include/fake_central_robot_navigation
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
  single_robot_cmd_vel_calculator
  ${catkin_LIBRARIES}
)

add_executable(multirobot_cmd_vel_calculator
  src/multirobot_cmd_vel_calculator.cpp
)
target_include_directories(multirobot_cmd_vel_calculator PUBLIC
  include/fake_central_robot_navigation
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(
  multirobot_cmd_vel_calculator
  tf_listener
  single_robot_cmd_vel_calculator
  ${catkin_LIBRARIES}
)

