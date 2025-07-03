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

# Keyboard
set(SERVO_KEYBOARD_INPUT_LIB keyboard_servo_component_lib)
set(SERVO_KEYBOARD_EXEC keyboard_servo_node)

# Joy
set(SERVO_JOY_INPUT_LIB joystick_servo_component_lib)
set(SERVO_JOY_EXEC joy_servo_node)

include_directories(
  include
)

#####################
## Component Nodes ##
#####################

# Add and export library to run as a ROS node component, and receive commands via topics
# Add executable for using a controller

# Keyboard
add_library(${SERVO_KEYBOARD_INPUT_LIB} SHARED src/keyboard_servo_component.cpp)
ament_target_dependencies(${SERVO_KEYBOARD_INPUT_LIB} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_KEYBOARD_INPUT_LIB} "moveit_servo::KeyboardServoComponent")


# Joy
add_library(${SERVO_JOY_INPUT_LIB} SHARED src/joystick_servo_component.cpp)
ament_target_dependencies(${SERVO_JOY_INPUT_LIB} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${SERVO_JOY_INPUT_LIB} "moveit_servo::JoyToServoComponent")


######################
## Executable Nodes ##
######################

# Executable that wraps the component

# Keyboard
add_executable(${SERVO_KEYBOARD_EXEC} src/keyboard_servo_node.cpp)
target_link_libraries(${SERVO_KEYBOARD_EXEC} keyboard_servo_component_lib)  # Use the component lib name here
ament_target_dependencies(${SERVO_KEYBOARD_EXEC} ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    ${SERVO_KEYBOARD_EXEC}
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(servo_keyboard_input src/servo_keyboard_input.cpp)
# target_link_libraries(servo_keyboard_input servo_keyboard_input)  # Use the component lib name here
ament_target_dependencies(servo_keyboard_input ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    servo_keyboard_input
  DESTINATION lib/${PROJECT_NAME}
)



# Joy
add_executable(${SERVO_JOY_EXEC} src/joystick_servo_node.cpp)
target_link_libraries(${SERVO_JOY_EXEC} joystick_servo_component_lib)  # Use the component lib name here
ament_target_dependencies(${SERVO_JOY_EXEC} ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    ${SERVO_JOY_EXEC}
  DESTINATION lib/${PROJECT_NAME}
)


#############
## Install ##
#############

# Components
install(
  TARGETS
    ${SERVO_KEYBOARD_INPUT_LIB} # Keyboard
    ${SERVO_JOY_INPUT_LIB}      # Joy
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
