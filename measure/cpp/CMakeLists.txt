cmake_minimum_required(VERSION 3.1)

project(cluster)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)

add_executable(cluster main.cpp)
target_link_libraries (cluster ${PCL_LIBRARIES})