cmake_minimum_required(VERSION 2.8.3)
project(create_vector)

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


add_executable(create_vector src/create_vector.cpp)
target_link_libraries(create_vector ${PCL_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS create_vector
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
