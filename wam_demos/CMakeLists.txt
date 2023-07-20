cmake_minimum_required(VERSION 2.8.3)
project(wam_demos)

find_package(catkin REQUIRED COMPONENTS
  wam_srvs
  wam_msgs
  rosbag
  geometry_msgs
)
catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  wam_srvs
  wam_msgs
  rosbag
  geometry_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_definitions(
   -std=c++11
 )
add_executable(teach  
    src/teach.cpp
)
target_link_libraries(teach ${catkin_LIBRARIES})

add_executable(play  
    src/play.cpp
)
target_link_libraries(play ${catkin_LIBRARIES})
