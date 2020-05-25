cmake_minimum_required(VERSION 2.8.3)
project(grvc_ef_tracker)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native" )

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  dvs_msgs
  roscpp
  std_msgs
)

find_package(cmake_modules REQUIRED)

catkin_package(INCLUDE_DIRS 
	${catkin_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(ef_tracker
  src/tracker.cpp
  src/trackerWrapper.cpp
)

target_link_libraries(ef_tracker ${catkin_LIBRARIES})

add_executable(ef_tracker_node src/main.cpp)
target_link_libraries(ef_tracker_node ef_tracker) 