cmake_minimum_required(VERSION 3.10)

project(point_cloud_compression)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(ZLIB REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(octree_compression "octree_compression.cpp")

add_executable(octree_uncompression "octree_uncompression.cpp")

add_executable(zlib_compression "zlib_compression.cpp" "zpipe.cpp")

add_executable(zlib_uncompression "zlib_compression.cpp" "zpipe.cpp")

target_link_libraries(point_cloud_compression PRIVATE ${PCL_LIBRARIES} ZLIB::ZLIB)
