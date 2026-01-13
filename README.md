# Sensor Fusion and Perception Stack

Multi-sensor perception system using Camera + LiDAR + Kalman Filter for autonomous vehicle perception.

## Features
- Camera-based object detection (YOLOv8)
- LiDAR-based clustering
- Sensor fusion (Camera + LiDAR)
- Kalman Filter multi-object tracking
- RViz visualization

## Tech Stack
- ROS2 Humble
- Gazebo Fortress
- OpenCV
- PCL
- Eigen

## Build
```bash
cd ~/ros2_ws
colcon build --packages-select sensor_fusion_stack
source install/setup.bash
```

## Author
Meet Jain - Northeastern University
