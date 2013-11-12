cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories("src")

file(GLOB HEADER "src/*.h"   "src/*.hpp")
file(GLOB SOURCE "src/*.cpp")

rosbuild_add_executable(${PROJECT_NAME} ${HEADER} ${SOURCE})
