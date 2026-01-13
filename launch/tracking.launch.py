from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    kalman_tracker = Node(
        package='sensor_fusion_stack',
        executable='kalman_tracker',
        name='kalman_tracker',
        output='screen',
        parameters=[{
            'max_association_distance': 2.0,
            'max_misses': 5,
            'min_hits': 3
        }]
    )
    
    return LaunchDescription([
        kalman_tracker
    ])
