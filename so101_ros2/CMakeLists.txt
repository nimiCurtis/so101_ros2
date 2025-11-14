cmake_minimum_required(VERSION 3.10...3.27)
project(so101_ros2)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
#   find_package(ament_clang_format REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
