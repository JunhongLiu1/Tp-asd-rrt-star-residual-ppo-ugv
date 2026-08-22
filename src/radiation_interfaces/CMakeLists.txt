cmake_minimum_required(VERSION 3.5)
project(radiation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlannerMetrics.msg"
  "msg/ControlMetrics.msg"
  "msg/RadiationMeasurement.msg"
  "msg/RiskMap.msg"
  "action/PlanRiskAwarePath.action"
  "srv/QueryRadiation.srv"
  "srv/QueryRadiationCost.srv"
  "srv/QueryPathRadiationCost.srv"
  "srv/QueryTerrainCost.srv"
  "srv/QueryFusionCost.srv"
  "srv/QueryPathScore.srv"
  "srv/QueryPathFusionMapCost.srv"
  "srv/QueryTimeAwarePathCost.srv"
  "srv/SelectBestTimeAwarePath.srv"
  "srv/TerrainQuery.srv"
  DEPENDENCIES std_msgs nav_msgs geometry_msgs
)

ament_package()
