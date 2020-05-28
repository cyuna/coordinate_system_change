cmake_minimum_required(VERSION 2.8.3)
project(revert_coordinate_system)

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


add_executable(revert_coordinate_system revert_coordinate_system.cpp)
target_link_libraries(revert_coordinate_system ${PCL_LIBRARIES} ${catkin_LIBRARIES})
install(TARGETS revert_coordinate_system
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )