cmake_minimum_required(VERSION 3.14)
project(PCL_Normal)

# PCL
find_package(PCL 1.8 REQUIRED)
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

add_executable(PCL_Normal main.cpp)

target_link_libraries(PCL_Normal
        ${PCL_LIBRARIES}
        ${EIGEN_LIBRARIES})