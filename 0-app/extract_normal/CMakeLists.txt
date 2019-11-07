cmake_minimum_required(VERSION 3.10)
project(extract_normal)

# PCL
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -std=c++14")

# Eigen library
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(extract_normal extract_normal.cpp)

target_link_libraries(extract_normal
        ${PCL_LIBRARIES}
        ${EIGEN_LIBRARIES})
