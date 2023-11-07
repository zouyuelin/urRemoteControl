cmake_minimum_required(VERSION 3.2.0)
project(ultrasound)

add_compile_options(-std=c++11)
add_compile_options(-std=c++14)

set(CMAKE_INCLUDE_SYSTEM_FLAG_CXX "-isystem")

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    sensor_msgs
    trajectory_msgs
    netft_utils
    cv_bridge
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(Eigen3 REQUIRED)

include_directories(include
                    include/function
                    /usr/include
                    SYSTEM
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${EIGEN3_INCLUDE_DIR}
                    ${Sophus_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
  DEPENDS OpenCV PCL
)

add_library(KINEMATIC_ src/function/ur_kine.cpp)
set_target_properties(KINEMATIC_ PROPERTIES COMPILE_DEFINITIONS "UR5_PARAMS")

# add_executable(ur_passive
#   src/function/Transformer.cpp
#   src/demo.cpp
# )
# add_dependencies(ur_passive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ur_passive ${catkin_LIBRARIES} KINEMATIC)

# add_executable(ur_autonomous
#   src/function/Transformer.cpp
#   src/ur_autonomous.cpp
# )
# add_dependencies(ur_autonomous ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(ur_autonomous ${catkin_LIBRARIES} KINEMATIC)

add_executable(tracking
  src/function/Transformer.cpp
  src/tracking.cpp
  src/function/robot.cpp
)
add_dependencies(tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracking ${catkin_LIBRARIES} KINEMATIC_)

add_executable(dragging
  src/function/Transformer.cpp
  src/dragging.cpp
  src/function/robot.cpp
)
add_dependencies(dragging ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dragging ${catkin_LIBRARIES} KINEMATIC_)

add_executable(joycontrol
  src/function/Transformer.cpp
  src/joycontrol.cpp
  src/function/robot.cpp
)
add_dependencies(joycontrol ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joycontrol ${catkin_LIBRARIES} KINEMATIC_)
