cmake_minimum_required(VERSION 3.10...3.27)

project(so101_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# # Install the mesh files from SO101/assets
# install(
# 	DIRECTORY
# 		SO101/assets/
# 	DESTINATION
# 		share/${PROJECT_NAME}/meshes
# 	FILES_MATCHING PATTERN "*.stl"
# )

# Install URDF files
install(
	DIRECTORY
		urdf/
	DESTINATION
		share/${PROJECT_NAME}/urdf
	FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro"
)

# Install other directories
install(
	DIRECTORY
		meshes
		config
		launch
		rviz
	DESTINATION
		share/${PROJECT_NAME}
	OPTIONAL
)

if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        # find_package(ament_clang_format REQUIRED)
        ament_lint_auto_find_test_dependencies()
endif()

ament_package()

