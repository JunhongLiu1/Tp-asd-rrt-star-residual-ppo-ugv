cmake_minimum_required(VERSION 3.5)

project(gazebo_radiation_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(
  gazebo_radiation_world_plugin
  SHARED
  src/radiation_world_plugin.cpp
)

ament_target_dependencies(
  gazebo_radiation_world_plugin
  gazebo_ros
  rclcpp
  std_msgs
  nav_msgs
)

target_link_libraries(
  gazebo_radiation_world_plugin
  ${GAZEBO_LIBRARIES}
)

install(
  TARGETS
  gazebo_radiation_world_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
