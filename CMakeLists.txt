cmake_minimum_required(VERSION 2.8.3)
project(autonomous_exploration)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  opt_utils
  internal_grid_map
  grid_map_ros
  path_transform_planner
  car_model
  iv_explore_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS  tf path_transform_planner internal_grid_map car_model iv_explore_msgs
  DEPENDS Boost
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(explore_client src/explore_client.cpp)
target_link_libraries(explore_client ${catkin_LIBRARIES})

add_executable(explore_server src/explore_server.cpp src/bfs_frontier_search.cpp)
target_link_libraries(explore_server ${catkin_LIBRARIES})

add_executable(get_map_server src/get_map_server.cpp)
target_link_libraries(get_map_server ${catkin_LIBRARIES})

catkin_add_gtest(explore_server_test
        tests/explore_server_test.cpp)

target_link_libraries(explore_server_test
        ${catkin_LIBRARIES}
        )