PROJECT(rgb_line_navigating_robot)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
ROSBUILD_INIT()

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

ROSBUILD_GENMSG()
ROSBUILD_GENSRV()

SET(libs roslib roscpp)

ROSBUILD_ADD_EXECUTABLE(rgb_line_navigating_robot
    src/rgb_line_navigating_robot.cpp)
TARGET_LINK_LIBRARIES(rgb_line_navigating_robot ${libs})
