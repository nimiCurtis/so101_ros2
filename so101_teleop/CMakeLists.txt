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

# Leader
set(LEADER_TELEOP_LIB leader_teleop_component_lib)
set(LEADER_TELEOP_EXEC leader_teleop_node)

include_directories(
  include
)

#####################
## Component Nodes ##
#####################

# Add and export library to run as a ROS node component, and receive commands via topics
# Add executable for using a controller

# Leader
add_library(${LEADER_TELEOP_LIB} SHARED src/leader_teleop_component.cpp)
ament_target_dependencies(${LEADER_TELEOP_LIB} ${THIS_PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(${LEADER_TELEOP_LIB} "so101_teleop::LeaderTeleopComponent")

######################
## Executable Nodes ##
######################

# Executable that wraps the component

# Leader
add_executable(${LEADER_TELEOP_EXEC} src/leader_teleop_node.cpp)
target_link_libraries(${LEADER_TELEOP_EXEC} ${LEADER_TELEOP_LIB})
ament_target_dependencies(${LEADER_TELEOP_EXEC} ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
    ${LEADER_TELEOP_EXEC}
  DESTINATION lib/${PROJECT_NAME}
)


#############
## Install ##
#############

# Components
install(
  TARGETS
    ${LEADER_TELEOP_LIB}        # Leader
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # find_package(ament_clang_format REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
