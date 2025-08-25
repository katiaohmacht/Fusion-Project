from launch import LaunchDescription
import os
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'pixel_format': 'mjpeg'
            }]
        ),
        Node(
            package='isaac_ros_yolov8',
            executable='yolov8_node',
            name='yolov8',
            parameters=[{
                'input_image_topic': '/image_raw'
            }]

        )
    ])


