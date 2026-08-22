cmake_minimum_required(VERSION 3.5)

project(risk_aware_planner_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(radiation_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(planner_core SHARED
  src/planner_core.cpp
)

add_library(pure_pursuit_core SHARED
  src/pure_pursuit_core.cpp
)
add_library(pid_core SHARED src/pid_core.cpp)
add_library(safety_gate_core SHARED src/safety_gate_core.cpp)
add_library(risk_map_core SHARED src/risk_map_core.cpp)
target_compile_features(safety_gate_core PUBLIC cxx_std_14)
target_include_directories(safety_gate_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(risk_map_core PUBLIC cxx_std_14)
target_include_directories(risk_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(pure_pursuit_core PUBLIC cxx_std_14)
target_include_directories(pure_pursuit_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(pid_core PUBLIC cxx_std_14)
target_include_directories(pid_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(planner_core PUBLIC cxx_std_14)

target_include_directories(planner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(planner_node
  src/planner_node.cpp
)

target_compile_features(planner_node PUBLIC cxx_std_14)

add_executable(path_follower_node
  src/path_follower_node.cpp
)

target_compile_features(path_follower_node PUBLIC cxx_std_14)

ament_target_dependencies(planner_node
  geometry_msgs
  nav_msgs
  radiation_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

ament_target_dependencies(path_follower_node
  geometry_msgs
  nav_msgs
  radiation_interfaces
  rclcpp
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_link_libraries(planner_node planner_core)
target_link_libraries(path_follower_node pure_pursuit_core pid_core)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS planner_core pure_pursuit_core pid_core safety_gate_core risk_map_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS planner_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS path_follower_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_planner_core
    test/test_planner_core.cpp
  )
  target_link_libraries(test_planner_core planner_core)
  target_compile_features(test_planner_core PUBLIC cxx_std_14)

  ament_add_gtest(test_edge_cost_model
    test/test_edge_cost_model.cpp
  )
  target_include_directories(test_edge_cost_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(test_edge_cost_model PUBLIC cxx_std_14)

  ament_add_gtest(test_pure_pursuit_core
    test/test_pure_pursuit_core.cpp
  )
  target_link_libraries(test_pure_pursuit_core pure_pursuit_core)
  target_compile_features(test_pure_pursuit_core PUBLIC cxx_std_14)
  ament_add_gtest(test_safety_gate_core test/test_safety_gate_core.cpp)
  target_link_libraries(test_safety_gate_core safety_gate_core)
  target_compile_features(test_safety_gate_core PUBLIC cxx_std_14)
  ament_add_gtest(test_risk_map_core test/test_risk_map_core.cpp)
  target_link_libraries(test_risk_map_core risk_map_core)
  target_compile_features(test_risk_map_core PUBLIC cxx_std_14)
  ament_add_gtest(test_pid_core test/test_pid_core.cpp)
  target_link_libraries(test_pid_core pid_core)
  target_compile_features(test_pid_core PUBLIC cxx_std_14)
endif()

ament_export_include_directories(include)
ament_export_libraries(planner_core pure_pursuit_core pid_core)

add_executable(
  radiation_online_mapper_node
  src/radiation_online_mapper_node.cpp
)

ament_target_dependencies(
  radiation_online_mapper_node
  rclcpp
  geometry_msgs
  nav_msgs
  radiation_interfaces
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(radiation_online_mapper_node risk_map_core)

install(
  TARGETS radiation_online_mapper_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(
  map_odom_alignment_node
  src/map_odom_alignment_node.cpp
)

ament_target_dependencies(
  map_odom_alignment_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(
  TARGETS map_odom_alignment_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

    
add_executable(planner_metrics_node
  src/planner_metrics_node.cpp
)

ament_target_dependencies(planner_metrics_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  radiation_interfaces
)

install(TARGETS
  planner_metrics_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})


add_executable(planner_metrics_recorder_node
  src/planner_metrics_recorder_node.cpp
)

ament_target_dependencies(planner_metrics_recorder_node
  rclcpp
  radiation_interfaces
)

install(TARGETS
  planner_metrics_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(cmd_vel_safety_node
  src/cmd_vel_safety_node.cpp
)

ament_target_dependencies(cmd_vel_safety_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)
target_link_libraries(cmd_vel_safety_node safety_gate_core)

install(TARGETS
  cmd_vel_safety_node
  DESTINATION lib/${PROJECT_NAME}
)
