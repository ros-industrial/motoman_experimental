cmake_minimum_required(VERSION 2.8.3)
project(motoman_robot_pkg_gen)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  industrial_robot_pkg_gen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

