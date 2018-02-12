cmake_minimum_required(VERSION 2.8.3)
project(ndtpso_slam)

if (BUILD_ROS_PACKAGE)
include(${CMAKE_CURRENT_SOURCE_DIR}/ros-build.cmake)
else()
include(${CMAKE_CURRENT_SOURCE_DIR}/test-build.cmake)
endif()
