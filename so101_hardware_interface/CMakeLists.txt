cmake_minimum_required(VERSION 3.10...3.27)
project(so101_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# Add the include directory for the header file
include_directories(include)

# Create the shared library for the hardware interface plugin
add_library(so101_hardware_bridge SHARED
  src/so101_hardware_bridge.cpp
)

# Link the library against its dependencies
ament_target_dependencies(so101_hardware_bridge
  rclcpp
  hardware_interface
  pluginlib
)

# Install the compiled library to the lib directory
install(TARGETS so101_hardware_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install the header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Register the plugin with pluginlib
pluginlib_export_plugin_description_file(hardware_interface so101_hardware_bridge_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
