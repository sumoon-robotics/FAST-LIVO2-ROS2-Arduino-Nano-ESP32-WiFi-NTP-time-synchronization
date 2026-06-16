from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('mvs_ros2_cam'),
        'config', 'cam_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='mvs_ros2_cam',
            executable='grab_trigger',
            name='mvs_trigger',
            output='screen',
            arguments=[config],
        )
    ])
