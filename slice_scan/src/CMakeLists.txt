cmake_minimum_required(VERSION 3.10)

project(slice_scan)

find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(slice_scan main.cpp Mesh.h Mesh.cpp)

target_link_libraries(slice_scan ${OpenCV_LIBRARIES})