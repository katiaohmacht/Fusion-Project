from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='yolov8n.pt'),
        DeclareLaunchArgument('conf', default_value='0.25'),
        DeclareLaunchArgument('device', default_value='cpu'),
        DeclareLaunchArgument('input_image_topic', default_value='/image_raw'),
        DeclareLaunchArgument('output_image_topic', default_value='/yolo/image_raw'),

        Node(
            package='yolo_overlay',
            executable='yolo_overlay_node',
            name='yolo_overlay',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'conf': LaunchConfiguration('conf'),
                'device': LaunchConfiguration('device'),
                'input_image_topic': LaunchConfiguration('input_image_topic'),
                'output_image_topic': LaunchConfiguration('output_image_topic'),
                'draw_labels': True,
                'draw_scores': True,
            }],
            output='screen'
        )
    ])



