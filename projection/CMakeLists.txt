cmake_minimum_required(VERSION 3.1)

project(measure)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(projection main.cpp utils.h utils.cpp)
target_link_libraries (projection ${PCL_LIBRARIES})
target_link_libraries (projection ${OpenCV_LIBRARIES})