cmake_minimum_required(VERSION 2.13)
set(CMAKE_BUILD_TYPE Release)

project(transformPointCloud)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(transformPointCloud transformPointCloud.cpp)
target_link_libraries (transformPointCloud ${PCL_LIBRARIES})

add_executable(transform transform.cpp)
target_link_libraries (transform ${PCL_LIBRARIES})