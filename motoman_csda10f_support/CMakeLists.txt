
cmake_minimum_required(VERSION 3.0.2)

project(motoman_csda10f_support)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
