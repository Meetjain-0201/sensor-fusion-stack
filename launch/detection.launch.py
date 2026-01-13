from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    camera_detector = Node(
        package='sensor_fusion_stack',
        executable='camera_detector.py',
        name='camera_detector',
        output='screen',
        parameters=[{
            'model_path': 'yolov8n.pt',
            'confidence_threshold': 0.3,  # Lowered to detect more
            'publish_annotated': True
        }]
    )
    
    return LaunchDescription([
        camera_detector
    ])
