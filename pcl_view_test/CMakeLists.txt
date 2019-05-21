cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 11)

project(pcl_view_test)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable (pcl_view_test pcl_view_test.cpp)
target_link_libraries (pcl_view_test ${PCL_LIBRARIES} ${OpenCV_LIBS})
