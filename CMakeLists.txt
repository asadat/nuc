cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(NUC_LIBRARIES GL GLU glut roscpp rostime rosconsole)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()
include_directories("./inlcude")
include_directories("/usr/include")

#include_directories("~/tag")
link_directories("/opt/ros/fuerte/lib")
include_directories("~")


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(NUC
src/NUC.cpp
src/NUC_node.cpp
src/CNode.cpp
src/DepthFirstStrategy.cpp
src/ShortCutStrategy.cpp
src/LawnmowerStrategy.cpp
src/MAV.cpp

include/NUC.h
include/CNode.h
include/DepthFirstStrategy.h
include/ShortCutStrategy.h
include/LawnmowerStrategy.h
include/TraversalStrategy.h
include/MAV.h
)

#rosbuild_add_executable(PositionVis
#include/PositionVis.h
#src/PositionVis.cpp
#)

target_link_libraries(NUC  ${NUC_LIBRARIES})

