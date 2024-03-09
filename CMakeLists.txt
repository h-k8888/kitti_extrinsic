cmake_minimum_required(VERSION 3.27)
project(kitti_extrinsic)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
add_executable(kitti_extrinsic main.cpp)
