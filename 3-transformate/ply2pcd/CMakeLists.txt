#CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
project(ply2pcd)

set(CMAKE_CXX_STANDARD 11)
find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(ply2pcd ply2pcd.cpp)
target_link_libraries(ply2pcd ${PCL_LIBRARIES})

add_executable(ply_show ply_show.cpp)
target_link_libraries(ply_show ${PCL_LIBRARIES})
