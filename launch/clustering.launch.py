from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    lidar_clustering = Node(
        package='sensor_fusion_stack',
        executable='lidar_clustering',
        name='lidar_clustering',
        output='screen',
        parameters=[{
            'voxel_leaf_size': 0.05,
            'ground_threshold': 0.2,
            'cluster_tolerance': 0.5,
            'min_cluster_size': 10,
            'max_cluster_size': 5000,
            'min_height': 0.1,
            'max_height': 3.0
        }]
    )
    
    return LaunchDescription([
        lidar_clustering
    ])
