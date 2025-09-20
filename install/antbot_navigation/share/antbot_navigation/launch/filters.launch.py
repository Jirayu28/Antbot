from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('antbot_navigation')
    cfg = os.path.join(pkg, 'config', 'laser_filters.yaml')

    return LaunchDescription([
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_filter',
            output='screen',
            parameters=[cfg],
            remappings=[('scan', '/scan'), ('scan_filtered', '/scan_filtered')],
        )
    ])