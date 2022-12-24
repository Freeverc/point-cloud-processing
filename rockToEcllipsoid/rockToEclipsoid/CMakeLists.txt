cmake_minimum_required(VERSION 3.10)

project(point_cluster)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(CGAL REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${CERES_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

add_executable(rockToEllipsolid main.cpp)

target_link_libraries(rockToEllipsolid ${CERES_LIBRARIES})
target_link_libraries(rockToEllipsolid CGAL::CGAL)