cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(supervoxel_clustering)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (lccp example_lccp_segmentation.cpp)
target_link_libraries (lccp ${PCL_LIBRARIES})

add_executable (cpc example_cpc_segmentation.cpp)
target_link_libraries (cpc ${PCL_LIBRARIES})

add_executable (lccp_test lccp_test.cpp)
target_link_libraries (lccp_test ${PCL_LIBRARIES})