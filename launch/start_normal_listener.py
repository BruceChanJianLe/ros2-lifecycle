from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
        return LaunchDescription([
            Node(package='ros2-lifecycle', executable='normal_listener', name='normal_listener', namespace='', output='screen')
        ])