cmake_minimum_required(VERSION 2.8.3)
project(coordinate_system_change)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
roscpp

)
catkin_package(CATKIN_DEPENDS
  roscpp
)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} )
include_directories(src ${EIGEN3_INCLUDE_DIR})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(coordinate_system_change coordinate_system_change.cpp)
target_link_libraries(coordinate_system_change ${PCL_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS coordinate_system_change
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )