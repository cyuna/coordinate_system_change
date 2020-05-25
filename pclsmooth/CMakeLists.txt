cmake_minimum_required(VERSION 2.8.3)
project(pclsmooth)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
roscpp

)
find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pclsmooth pclsmooth.cpp)
target_link_libraries(pclsmooth ${PCL_LIBRARIES} ${catkin_LIBRARIES})