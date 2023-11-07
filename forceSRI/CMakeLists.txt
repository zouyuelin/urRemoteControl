cmake_minimum_required(VERSION 3.0.2)
project(forceSRI)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES forceSRI
  CATKIN_DEPENDS roscpp rospy std_msgs
  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
    script/sri_sensor.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
