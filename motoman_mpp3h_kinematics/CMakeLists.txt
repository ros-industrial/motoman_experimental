cmake_minimum_required(VERSION 2.8.3)
project(motoman_mpp3h_kinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mpp3h_kinematics
  CATKIN_DEPENDS tf
  DEPENDS Boost
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(mpp3h_kinematics
 src/mpp3h_kinematics.cpp
)
target_link_libraries(mpp3h_kinematics
  ${catkin_LIBRARIES}
)

add_executable(mpp3h_state_publisher
 src/mpp3h_state_publisher.cpp
)
target_link_libraries(mpp3h_state_publisher
  mpp3h_kinematics
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)

  catkin_add_gtest(kinematic_test test/kinematic_test.cpp)
  target_link_libraries(kinematic_test
    mpp3h_kinematics
    ${catkin_LIBRARIES}
  )

endif()

install(TARGETS mpp3h_state_publisher
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS mpp3h_kinematics
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
