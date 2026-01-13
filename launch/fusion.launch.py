from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    sensor_fusion = Node(
        package='sensor_fusion_stack',
        executable='sensor_fusion',
        name='sensor_fusion',
        output='screen',
        parameters=[{
            'cluster_tolerance': 0.5,
            'min_cluster_size': 10,
            'max_cluster_size': 5000,
            'association_threshold': 0.3
        }]
    )
    
    return LaunchDescription([
        sensor_fusion
    ])
