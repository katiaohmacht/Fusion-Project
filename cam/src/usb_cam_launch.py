#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    # Component container to hold composable nodes
    image_proc_container = Node(
        package='rclcpp_components',
        executable='component_container_mt',
        name='image_proc_container',
        output='screen'
    )

    # Load ImageFormatConverterNode as a composable node
    load_image_converter = LoadComposableNodes(
        target_container='image_proc_container',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_image_proc',
                plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                name='image_format_converter',
                remappings=[
                    ('image_raw', '/image_raw'),  # input topic
                    ('image', '/image')           # output topic
                ],
                parameters=[{
                    'encoding_desired': 'bgr8'    # convert to BGR8
                }]
            )
        ]
    )

    return LaunchDescription([
        image_proc_container,
        load_image_converter
    ])



