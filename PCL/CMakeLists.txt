cmake_minimum_required(VERSION 3.5)
project(PCL)
set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl main.cpp)

target_link_libraries (pcl ${PCL_LIBRARIES})



