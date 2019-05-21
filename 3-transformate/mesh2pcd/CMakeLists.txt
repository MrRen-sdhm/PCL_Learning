cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "-O3 -fopenmp -fPIC -Wno-deprecated -Wenum-compare -Wno-ignored-attributes -std=c++11")

project(mesh2pcd)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (mesh_single_view mesh_single_view.cpp)
target_link_libraries (mesh_single_view ${PCL_LIBRARIES})

add_executable (mesh_sampling mesh_sampling.cpp)
target_link_libraries (mesh_sampling ${PCL_LIBRARIES})

add_executable (mesh2pcd mesh2pcd.cpp)
target_link_libraries (mesh2pcd ${PCL_LIBRARIES})

add_executable (pcd_viewer pcd_viewer.cpp)
target_link_libraries (pcd_viewer ${PCL_LIBRARIES})
