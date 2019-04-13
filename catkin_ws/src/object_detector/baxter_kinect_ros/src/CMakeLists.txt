cmake_minimum_required(VERSION 3.13)
project(src)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)

add_executable(src
        baxter_kinect_ros_node.cpp
        processPoint.cpp
        processPoint.hpp)
