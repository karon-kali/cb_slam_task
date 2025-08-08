from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = os.path.join(get_package_share_directory('cb_slam'), 'param')
    params_file = os.path.join(pkg_share, 'localization_score_params.yaml')

    return LaunchDescription([
        Node(
            package='cb_slam',
            executable='localization_score',
            name='localization_score',
            output='screen',
            parameters=[params_file],
        ),
    ])
