cmake_minimum_required(VERSION 3.22)
project(gpr_perception)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(gpr_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(${PROJECT_NAME}_core
  src/occupancy_grid_mapper.cpp
  src/perception_pipeline.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core gpr_common::gpr_common)

add_library(${PROJECT_NAME}_ros
  src/perception_bridge.cpp
  src/ros_conversions.cpp
)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}_core
  gpr_common::gpr_common
  tf2::tf2)
ament_target_dependencies(${PROJECT_NAME}_ros
  rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_occupancy_grid_mapper test/test_occupancy_grid_mapper.cpp)
  if(TARGET test_occupancy_grid_mapper)
    target_link_libraries(test_occupancy_grid_mapper ${PROJECT_NAME}_core gpr_common::gpr_common)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(gpr_perception_core gpr_perception_ros)
ament_export_dependencies(gpr_common rclcpp sensor_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_package()
