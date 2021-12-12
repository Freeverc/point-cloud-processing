cmake_minimum_required(VERSION 3.10)

project(point_cloud_compression)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_compression main.cpp  )
# add_executable(point_cluster sc.cpp)

target_link_libraries(point_cloud_compression ${PCL_LIBRARIES})