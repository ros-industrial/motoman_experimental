cmake_minimum_required(VERSION 3.0.2)
project(motoman_robot_pkg_gen)

find_package(catkin REQUIRED COMPONENTS
  industrial_robot_pkg_gen
)

catkin_package(CATKIN_DEPENDS industrial_robot_pkg_gen)

catkin_python_setup()

