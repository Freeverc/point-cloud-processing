cmake_minimum_required(VERSION 3.10)

project(rock_smoothing)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

set(VCG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/vcglib)
#cmake_policy(SET CMP0074 NEW)
find_package(Eigen3 REQUIRED)

find_package(VCG REQUIRED)
if(VCG_FOUND)
	message(STATUS "Find VCG lib")
	add_definitions(${VCG_DEFINITIONS})
endif()

# find_path(VCGLIB_INCLUDE_DIRS   "vcg/complex/complex.h" 
#                                 "vcg/complex/algorithms/update/topology.h"
#                                 "vcg/complex/algorithms/update/normal.h"
#                                 "vcg/complex/algorithms/clean.h"
#                                 "vcg/complex/algorithms/smooth.h"
#                                 "wrap/io_trimesh/import.h"
#                                 "wrap/io_trimesh/export_ply.h")
# add_definitions(${PCL_DEFINITIONS})

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${VCG_INCLUDE_DIRS})

add_definitions(${VCG_DEFINITIONS})

add_executable(rock_smoothing rock_smoothing.cpp)

# target_include_directories(rock_smoothing PRIVATE ${VCGLIB_INCLUDE_DIRS})