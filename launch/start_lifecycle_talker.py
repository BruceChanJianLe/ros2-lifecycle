from launch import LaunchDescription
from launch_ros.actions import LifecycleNode


def generate_launch_description():
        return LaunchDescription([
            LifecycleNode(package='ros2-lifecycle', executable='lifecycle_talker', name='lc_talker', namespace='', output='screen')
        ])