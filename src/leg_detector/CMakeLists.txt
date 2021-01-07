cmake_minimum_required(VERSION 3.5)

project(leg_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set (OpenCV_DIR /usr/local/include)
include_directories(include ${OpenCV_INCLUDE_DIR})


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(leg_detector_msgs REQUIRED)
find_package(OpenCV 3.4.12 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)


# add the executables
add_executable (detect_leg_clusters
                src/detect_leg_clusters.cpp
                src/laser_processor.cpp
                src/cluster_features.cpp
                include/leg_detector/laser_processor.h
                include/leg_detector/cluster_features.h
                )
ament_target_dependencies(detect_leg_clusters rclcpp leg_detector_msgs OpenCV sensor_msgs tf2 tf2_geometry_msgs tf2_ros visualization_msgs) 

add_executable (local_occupancy_grid_mapping
                src/laser_processor.cpp
                src/local_occupancy_grid_mapping.cpp
                include/leg_detector/laser_processor.h
                )
ament_target_dependencies(local_occupancy_grid_mapping rclcpp leg_detector_msgs OpenCV sensor_msgs tf2 tf2_ros visualization_msgs nav_msgs tf2_geometry_msgs)
add_executable (inflated_human_scan
                src/inflated_human_scan.cpp
                )
ament_target_dependencies(inflated_human_scan rclcpp leg_detector_msgs sensor_msgs message_filters)

install (TARGETS
  detect_leg_clusters
  local_occupancy_grid_mapping
  inflated_human_scan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rosbag DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python executables
install(PROGRAMS
  scripts/joint_leg_tracker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
