cmake_minimum_required(VERSION 3.8)
project(scout_gazebo_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

set(dependencies
  "geometry_msgs"
  "nav_msgs"
  "rclcpp"
  "sensor_msgs"
  "tf2"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY
    launch
    models
    worlds
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()

#find_package(catkin REQUIRED COMPONENTS    
#    roslaunch 
#    roslint 
#    roscpp 
#    std_msgs
#    sensor_msgs
#    geometry_msgs
#    tf2 
#    tf2_ros
#)
#catkin_package(
#  INCLUDE_DIRS include
##   LIBRARIES 
#  CATKIN_DEPENDS roscpp sensor_msgs
##   DEPENDS Boost
#)

###########
## Build ##
###########

#include_directories(
#    include
#    ${catkin_INCLUDE_DIRS}
#)

#add_library(scout_gazebo STATIC src/scout_skid_steer.cpp)
#target_link_libraries(scout_gazebo ${catkin_LIBRARIES})

#add_executable(scout_skid_steer_controller src/scout_skid_steer_controller.cpp)
#target_link_libraries(scout_skid_steer_controller scout_gazebo ${catkin_LIBRARIES})

