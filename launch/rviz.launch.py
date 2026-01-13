import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    pkg_name = 'sensor_fusion_stack'
    pkg_share = get_package_share_directory(pkg_name)
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([rviz])
