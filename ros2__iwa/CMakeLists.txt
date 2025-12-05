cmake_minimum_required(VERSION 3.5)
project(ros2_iiwa)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY urdf meshes config launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
