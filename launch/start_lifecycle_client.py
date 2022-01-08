from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
        return LaunchDescription([
            Node(package='ros2-lifecycle', executable='lc_client', name='lifecycle_service_client', output='screen')
        ])