cmake_minimum_required(VERSION 3.8)
project(sensor_fusion_stack)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ALL dependencies FIRST
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(image_transport REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(include ${PCL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

# Dataset recorder executable
add_executable(dataset_recorder src/dataset_recorder.cpp)
ament_target_dependencies(dataset_recorder
  rclcpp
  sensor_msgs
  gazebo_msgs
  cv_bridge
  pcl_ros
  pcl_conversions
  image_transport
)
target_link_libraries(dataset_recorder ${PCL_LIBRARIES} ${OpenCV_LIBS})

# LiDAR clustering executable
add_executable(lidar_clustering src/lidar_clustering.cpp)
ament_target_dependencies(lidar_clustering
  rclcpp
  sensor_msgs
  visualization_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
)
target_link_libraries(lidar_clustering ${PCL_LIBRARIES})

# Sensor fusion executable
add_executable(sensor_fusion src/sensor_fusion.cpp)
ament_target_dependencies(sensor_fusion
  rclcpp
  sensor_msgs
  vision_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
)
target_link_libraries(sensor_fusion ${PCL_LIBRARIES} ${OpenCV_LIBS})

# Kalman tracker executable
add_executable(kalman_tracker src/kalman_tracker.cpp)
ament_target_dependencies(kalman_tracker
  rclcpp
  visualization_msgs
  geometry_msgs
)
target_link_libraries(kalman_tracker Eigen3::Eigen)

# Install executables
install(TARGETS
  dataset_recorder
  lidar_clustering
  sensor_fusion
  kalman_tracker
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  launch
  urdf
  worlds
  rviz
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# ament_package() MUST BE LAST
ament_package()
