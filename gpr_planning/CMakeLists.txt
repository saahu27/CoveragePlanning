cmake_minimum_required(VERSION 3.22)
project(gpr_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(gpr_common REQUIRED)
find_package(gpr_metrics REQUIRED)
find_package(gpr_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(GPR_PLANNING_CORE_SRCS
  src/boustrophedon_planner.cpp
  src/path_invalidator.cpp
  src/segment_catalog.cpp
  src/oarp_lite_planner.cpp
  src/boustrophedon_sequencer.cpp
  src/astar_grid_planner.cpp
  src/heuristic_atsp_solver.cpp
  src/sequence_solver_utils.cpp
  src/create_sequence_solver.cpp
)

# OR-Tools prebuilt is installed in Docker at /opt/ortools (amd64 + arm64 via install-ortools.sh).
set(GPR_ORTOOLS_PREFIX "/opt/ortools")
if(EXISTS "${GPR_ORTOOLS_PREFIX}/lib/cmake/ortools/ortoolsConfig.cmake")
  list(PREPEND CMAKE_PREFIX_PATH "${GPR_ORTOOLS_PREFIX}")
  find_package(ortools CONFIG REQUIRED)
  message(STATUS "gpr_planning: OR-Tools found at ${GPR_ORTOOLS_PREFIX}, enabling OrToolsAtspSolver")
  list(APPEND GPR_PLANNING_CORE_SRCS src/or_tools_atsp_solver.cpp)
  set(GPR_HAS_ORTOOLS 1)
else()
  find_package(ortools CONFIG QUIET)
  if(ortools_FOUND)
    message(STATUS "gpr_planning: OR-Tools found, enabling OrToolsAtspSolver")
    list(APPEND GPR_PLANNING_CORE_SRCS src/or_tools_atsp_solver.cpp)
    set(GPR_HAS_ORTOOLS 1)
  else()
    message(WARNING "gpr_planning: OR-Tools not found; using heuristic ATSP solver only")
    set(GPR_HAS_ORTOOLS 0)
  endif()
endif()

add_library(${PROJECT_NAME}_core ${GPR_PLANNING_CORE_SRCS})
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC gpr_common::gpr_common)
target_compile_definitions(${PROJECT_NAME}_core PUBLIC GPR_HAS_ORTOOLS=${GPR_HAS_ORTOOLS})
if(GPR_HAS_ORTOOLS)
  target_link_libraries(${PROJECT_NAME}_core PRIVATE ortools::ortools)
endif()

add_library(${PROJECT_NAME}_ros
  src/planning_engine.cpp
  src/planning_ops.cpp
  src/planning_worker.cpp
  src/ros_io_facade.cpp
  src/coverage_report_conversions.cpp
)
target_include_directories(${PROJECT_NAME}_ros PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}_core gpr_common::gpr_common gpr_metrics::gpr_metrics)
ament_target_dependencies(${PROJECT_NAME}_ros
  rclcpp geometry_msgs nav_msgs visualization_msgs gpr_msgs)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_ros
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_boustrophedon test/test_boustrophedon_planner.cpp)
  if(TARGET test_boustrophedon)
    target_link_libraries(test_boustrophedon ${PROJECT_NAME}_core gpr_common::gpr_common)
  endif()
  ament_add_gtest(test_path_invalidator test/test_path_invalidator.cpp)
  if(TARGET test_path_invalidator)
    target_link_libraries(test_path_invalidator ${PROJECT_NAME}_core gpr_common::gpr_common)
  endif()
  ament_add_gtest(test_segment_catalog test/test_segment_catalog.cpp)
  if(TARGET test_segment_catalog)
    target_link_libraries(test_segment_catalog ${PROJECT_NAME}_core gpr_common::gpr_common)
  endif()
  ament_add_gtest(test_oarp_lite test/test_oarp_lite_planner.cpp)
  if(TARGET test_oarp_lite)
    target_link_libraries(test_oarp_lite ${PROJECT_NAME}_core gpr_common::gpr_common)
  endif()
  ament_add_gtest(test_boustrophedon_sequencer test/test_boustrophedon_sequencer.cpp)
  if(TARGET test_boustrophedon_sequencer)
    target_link_libraries(
      test_boustrophedon_sequencer ${PROJECT_NAME}_core gpr_common::gpr_common)
  endif()
  ament_add_gtest(test_executed_trajectory test/test_executed_trajectory.cpp)
  if(TARGET test_executed_trajectory)
    target_link_libraries(
      test_executed_trajectory ${PROJECT_NAME}_ros gpr_common::gpr_common gpr_metrics::gpr_metrics)
    ament_target_dependencies(test_executed_trajectory rclcpp nav_msgs geometry_msgs visualization_msgs gpr_msgs)
  endif()
  ament_add_gtest(test_planning_engine_sequencer test/test_planning_engine_sequencer.cpp)
  if(TARGET test_planning_engine_sequencer)
    target_link_libraries(
      test_planning_engine_sequencer ${PROJECT_NAME}_ros gpr_common::gpr_common gpr_metrics::gpr_metrics)
    ament_target_dependencies(test_planning_engine_sequencer rclcpp nav_msgs geometry_msgs visualization_msgs gpr_msgs)
  endif()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(gpr_planning_core gpr_planning_ros)
ament_export_dependencies(gpr_common gpr_metrics gpr_msgs rclcpp geometry_msgs nav_msgs visualization_msgs)
ament_package(CONFIG_EXTRAS cmake/gpr_planning-extras.cmake)
