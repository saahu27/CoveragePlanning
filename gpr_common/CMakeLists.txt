cmake_minimum_required(VERSION 3.22)
project(gpr_common)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/grid_map.cpp
  src/coverage_types.cpp
  src/swath_coverage.cpp
  src/scan_region.cpp
  src/polygon_geometry.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(gpr_common)
add_library(gpr_common::gpr_common ALIAS gpr_common)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gpr_common test/test_gpr_common.cpp)
  if(TARGET test_gpr_common)
    target_link_libraries(test_gpr_common ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_polygon_geometry test/test_polygon_geometry.cpp)
  if(TARGET test_polygon_geometry)
    target_link_libraries(test_polygon_geometry ${PROJECT_NAME})
  endif()
  ament_add_gtest(test_swath_coverage test/test_swath_coverage.cpp)
  if(TARGET test_swath_coverage)
    target_link_libraries(test_swath_coverage ${PROJECT_NAME})
  endif()
endif()

ament_package()
