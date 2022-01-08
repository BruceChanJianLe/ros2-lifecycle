from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
        return LaunchDescription([
            LifecycleNode(package='ros2-lifecycle', executable='lifecycle_talker', name='lc_talker', namespace='', output='screen'),
            Node(package='ros2-lifecycle', executable='normal_listener', name='normal_listener', namespace='', output='screen'),
            Node(package='ros2-lifecycle', executable='lc_client', name='lifecycle_service_client', output='screen')
        ])