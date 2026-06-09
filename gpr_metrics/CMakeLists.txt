cmake_minimum_required(VERSION 3.22)
project(gpr_metrics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gpr_common REQUIRED)

add_library(${PROJECT_NAME}
  src/coverage_baseline.cpp
  src/coverage_accountant.cpp
  src/coverage_reporter.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC gpr_common::gpr_common)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(gpr_metrics)
ament_export_dependencies(gpr_common)
add_library(gpr_metrics::gpr_metrics ALIAS gpr_metrics)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_coverage_accountant test/test_coverage_accountant.cpp)
  if(TARGET test_coverage_accountant)
    target_link_libraries(test_coverage_accountant ${PROJECT_NAME} gpr_common::gpr_common)
  endif()
endif()

ament_package()
