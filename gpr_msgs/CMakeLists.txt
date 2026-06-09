cmake_minimum_required(VERSION 3.22)
project(gpr_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CoverageSegment.msg"
  "msg/CoverageJob.msg"
  "msg/Schedule.msg"
  "msg/UncoveredRegion.msg"
  "msg/CoverageReport.msg"
  DEPENDENCIES geometry_msgs nav_msgs std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
