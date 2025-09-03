from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    return LaunchDescription([

        ComposableNodeContainer(
            name='image_proc_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[

                ComposableNode(
                    package='isaac_ros_image_proc',
                    plugin='nvidia::isaac_ros::image_proc::ImageFormatConverterNode',
                    name='image_format_converter',
                    remappings=[
                        ('image_raw', '/image'),
                        ('camera_info', '/camera_info')
                    ]
                ),
            ],
            output='screen'
        )
    ])

