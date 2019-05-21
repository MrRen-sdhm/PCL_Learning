cmake_minimum_required(VERSION 2.8)

project(PCL_Kinect2)
set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} $ENV{HOME}/freenect2/lib/cmake/freenect2)
set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(PkgConfig REQUIRED)
FIND_PACKAGE(freenect2 REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories("/usr/include/libusb-1.0/")
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${freenect2_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

add_executable(PCL_Kinect2 main.cpp)
target_link_libraries(PCL_Kinect2  ${freenect2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
