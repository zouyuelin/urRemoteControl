cmake_minimum_required(VERSION 2.8.3)
project(masterSlaveOmega)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES assembler_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++14")


find_package(Eigen3 REQUIRED)

set(FollowDrag /home/yuelinzou/rosSpace/URControl/src/follow_drag)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    /usr/include
    SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${FollowDrag}/include
# ${catkin_INCLUDE_DIRS}
)

set(DHDLIB
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.so.3
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.so.3
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdhd.a
    ${CMAKE_CURRENT_SOURCE_DIR}/lib/libdrd.a)
find_library(USB1LIB  libusb-1.0.so)

add_library(KINEMATIC_MASTER src/function/ur_kine.cpp)
set_target_properties(KINEMATIC_MASTER PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")

add_executable(haptic_ros_driver
  src/omega/haptic_ros_driver.cpp
  src/omega/HapticDevice.cpp
)
add_dependencies(haptic_ros_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(haptic_ros_driver
  ${DHDLIB}
  ${DRDLIB}
  ${USB1LIB}
  ${catkin_LIBRARIES}
)

add_executable(sphereExample
  src/function/Transformer.cpp
  src/sphere.cpp
  src/function/robot.cpp
)
add_dependencies(sphereExample ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sphereExample
  ${DHDLIB}
  ${DRDLIB}
  ${USB1LIB}
  ${catkin_LIBRARIES}
  KINEMATIC_MASTER
)

add_executable(feedback
  src/function/Transformer.cpp
  src/dragging.cpp
  src/function/robot.cpp
)
add_dependencies(feedback ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(feedback
  ${DHDLIB}
  ${DRDLIB}
  ${USB1LIB}
  ${catkin_LIBRARIES}
  KINEMATIC_MASTER
)

