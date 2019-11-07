cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(recon_marchingCubes)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (recon_marchingCubes recon_marchingCubes.cpp)
target_link_libraries (recon_marchingCubes ${PCL_LIBRARIES})
