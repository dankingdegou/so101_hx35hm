cmake_minimum_required(VERSION 3.8)
project(so101_moveit_octomap_updater)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(moveit_core REQUIRED)
find_package(octomap REQUIRED)

add_library(so101_moveit_octomap_updater SHARED
  src/pointcloud_octomap_updater.cpp
)
target_compile_features(so101_moveit_octomap_updater PRIVATE cxx_std_17)

ament_target_dependencies(so101_moveit_octomap_updater
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  pluginlib
  moveit_ros_occupancy_map_monitor
  moveit_core
  octomap
)

target_include_directories(so101_moveit_octomap_updater PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(
  moveit_ros_occupancy_map_monitor
  so101_octomap_updater_plugins.xml
)

install(TARGETS so101_moveit_octomap_updater
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES so101_octomap_updater_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

