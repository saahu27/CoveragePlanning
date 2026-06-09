cmake_minimum_required(VERSION 3.22)
project(gpr_mission)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(gpr_common REQUIRED)
find_package(gpr_perception REQUIRED)
find_package(gpr_planning REQUIRED)
find_package(gpr_control REQUIRED)
find_package(gpr_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(gpr_mission_node
  src/gpr_mission_node.cpp
  src/mission_behaviors.cpp
  src/mission_context.cpp
  src/coverage_stack.cpp
  src/planning_job_utils.cpp
)
target_include_directories(gpr_mission_node PUBLIC include)
ament_target_dependencies(gpr_mission_node
  gpr_common
  gpr_perception
  gpr_planning
  gpr_control
  rclcpp
  behaviortree_cpp
  ament_index_cpp
  tf2
  tf2_geometry_msgs
  tf2_ros
  geometry_msgs
  std_msgs
  gpr_msgs
)
target_link_libraries(gpr_mission_node
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2::tf2)

install(TARGETS gpr_mission_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY bt_xml/ DESTINATION share/${PROJECT_NAME}/bt_xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
