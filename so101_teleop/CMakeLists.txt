cmake_minimum_required(VERSION 3.10...3.27)
project(so101_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()


set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2_eigen
  trajectory_msgs
)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()



# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
moveit_package()

set(TARGET_KEYBOARD so101_keyboard_teleop)
set(SERVO_CONTROLLER_INPUT servo_controller_input)
set(SERVO_KEYBOARD_INPUT_LIB keyboard_servo_component_lib)
set(SERVO_KEYBOARD_EXEC keyboard_servo_input)

include_directories(
  include
)

#####################
## Component Nodes ##
#####################

# Add and export library to run as a ROS node component, and receive commands via topics
# Add executable for using a controller
add_library(${SERVO_CONTROLLER_INPUT} SHARED src/joystick_servo_example.cpp)
ament_target_dependencies(${SERVO_CONTROLLER_INPUT} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_CONTROLLER_INPUT} "moveit_servo::JoyToServoPub")

add_library(${SERVO_KEYBOARD_INPUT_LIB} SHARED src/keyboard_servo_component.cpp)
ament_target_dependencies(${SERVO_KEYBOARD_INPUT_LIB} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_KEYBOARD_INPUT_LIB} "moveit_servo::KeyboardServoComponent")

######################
## Executable Nodes ##
######################

# Executable that wraps the component
add_executable(keyboard_servo_input src/keyboard_servo_main.cpp)
target_link_libraries(keyboard_servo_input keyboard_servo_component_lib)  # Use the component lib name here
ament_target_dependencies(keyboard_servo_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    ${SERVO_KEYBOARD_EXEC}
  DESTINATION lib/${PROJECT_NAME}
)


# add_executable(${TARGET_KEYBOARD} src/so101_keyboard_teleop.cpp)

# target_include_directories(${TARGET_KEYBOARD} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )
# target_compile_features(${TARGET_KEYBOARD} PUBLIC c_std_99 cxx_std_17)

# ament_target_dependencies(${TARGET_KEYBOARD}
#   rclcpp
#   std_msgs
# )

# install(TARGETS ${TARGET_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )


#############
## Install ##
#############

install(
  TARGETS
    ${SERVO_KEYBOARD_INPUT_LIB}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# Install Libraries
install(
  TARGETS
    ${SERVO_CONTROLLER_INPUT}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


# Install include, launch, config directories
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
