from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import math

def generate_launch_description():
    """
    Hardcoded transforms.
    """

    # x y z yaw pitch roll parent_frame child_frame
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_transform',
        arguments=['0.02', '0.0', '-0.1','0.0', '0.0', '0.0', 'base_link', 'camera_link']
    )
    
    radar_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='radar_transform',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'radar_link']
    )
    
    radar_camera_overlay = Node(
        package='radar_camera_fusion',  
        executable='fusion_node',
        name='radar_camera_overlay',
        parameters=[{
            'camera_frame': 'camera_link',
            'radar_frame': 'radar_link',
            'point_size': 4,
            'point_color_r': 0,
            'point_color_g': 255,
            'point_color_b': 0,
            'max_distance': 50.0
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_transform,
        radar_transform,
        radar_camera_overlay
    ])
