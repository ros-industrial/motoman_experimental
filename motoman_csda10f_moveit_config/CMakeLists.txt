cmake_minimum_required(VERSION 2.8.3)
project(motoman_csda10f_moveit_config)

find_package(catkin REQUIRED
  trac_ik_kinematics_plugin             ## Kinematics solver for both arms
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
)

catkin_package(
  CATKIN_DEPENDS 
    trac_ik_kinematics_plugin 
    moveit_core
    moveit_ros_planning_interface
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
