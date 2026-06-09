cmake_minimum_required(VERSION 3.22)
project(gpr_bringup)

find_package(ament_cmake REQUIRED)
install(DIRECTORY config configs launch worlds rviz urdf DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
