cmake_minimum_required(VERSION 3.8)
project(scout_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
# For control
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(velocity_controllers REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    config
    urdf
    rviz
    launch
    meshes
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
