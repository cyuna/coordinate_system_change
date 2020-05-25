cmake_minimum_required(VERSION 2.8.3)
project(test2)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
roscpp
geometry_msgs
)
find_package(PCL 1.8 REQUIRED COMPONENTS common visualization)



add_executable(test2 src/test2.cpp)
target_link_libraries(test2 ${catkin_LIBRARIES} ${PCL_LIBRARIES})

include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES test2
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(

)

