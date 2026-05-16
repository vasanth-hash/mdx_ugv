cmake_minimum_required(VERSION 3.8)
project(mdx_ugv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
# find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(costmap_converter)
find_package(geographic_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(move_model SHARED plugins/move_model.cpp)
target_link_libraries(move_model PUBLIC ignition-gazebo6::ignition-gazebo6)

target_include_directories(move_model PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${ignition-gazebo6_INCLUDE_DIRS}
)

add_executable(
  edge_pilot_telemetry_handler src/edge_pilot_telemetry_handler.cpp)

add_executable(
  interpolator src/interpolator.cpp)

add_executable(
  dynamic_obstacle_processor src/dynamic_obstacle_processor.cpp)

add_executable(
  telemetry_handler_wp_manager src/telemetry_handler_wp_manager.cpp)

# add_executable(
#   waypoint_manager src/waypoint_manager.cpp)

add_executable(
  waypoint_manager_interpolater src/wp_manager_interpolater.cpp)

add_executable(
  lidar_zone_detector src/lidar_zone_detector.cpp)

ament_target_dependencies(interpolator rclcpp rclcpp_action tf2 tf2_geometry_msgs nav_msgs nav2_msgs geometry_msgs visualization_msgs)

ament_target_dependencies(dynamic_obstacle_processor rclcpp sensor_msgs visualization_msgs sensor_msgs costmap_converter)

# ament_target_dependencies(waypoint_manager rclcpp rclcpp_action std_msgs geometry_msgs nav2_msgs visualization_msgs tf2_ros tf2_geometry_msgs)

ament_target_dependencies(edge_pilot_telemetry_handler rclcpp sensor_msgs geometry_msgs nav2_msgs tf2)

ament_target_dependencies(waypoint_manager_interpolater rclcpp rclcpp_action sensor_msgs geometry_msgs nav2_msgs visualization_msgs tf2_ros tf2_geometry_msgs)

ament_target_dependencies(telemetry_handler_wp_manager rclcpp rclcpp_action sensor_msgs geometry_msgs nav2_msgs visualization_msgs tf2 tf2_ros tf2_geometry_msgs geographic_msgs)

ament_target_dependencies(lidar_zone_detector rclcpp sensor_msgs pcl_conversions)

install(TARGETS
 interpolator
 DESTINATION lib/${PROJECT_NAME})

install(TARGETS
 dynamic_obstacle_processor
 DESTINATION lib/${PROJECT_NAME})

# install(TARGETS
#  waypoint_manager
#  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
 waypoint_manager_interpolater
 DESTINATION lib/${PROJECT_NAME})

install(TARGETS
 edge_pilot_telemetry_handler
 DESTINATION lib/${PROJECT_NAME})

install(TARGETS
 telemetry_handler_wp_manager
 DESTINATION lib/${PROJECT_NAME})

install(TARGETS
 lidar_zone_detector
 DESTINATION lib/${PROJECT_NAME})


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

install(DIRECTORY behavior_trees/
  DESTINATION share/${PROJECT_NAME}/behavior_trees
)

install(PROGRAMS 
  scripts/heading_node.py
  scripts/pose_frame_id_adder.py
  scripts/obstacle_processor.py
  scripts/dynamic_obstacle_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS move_model
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY
    behavior_trees
    config
    launch 
    description 
    rviz 
    plugins
    scripts
    worlds
  DESTINATION 
    share/${PROJECT_NAME}
)

ament_package()
