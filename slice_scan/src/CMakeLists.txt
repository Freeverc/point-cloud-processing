cmake_minimum_required(VERSION 3.10)

project(slice_scan)

find_package(OpenCV REQUIRED)

find_package( OpenMP REQUIRED)

if(OPENMP_FOUND)
message("OPENMP FOUND")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories(${OpenCV_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

add_executable(slice_scan "main.cpp" "Geometry.h" "Mesh.h" "Mesh.cpp")

target_link_libraries(slice_scan ${OpenCV_LIBRARIES})