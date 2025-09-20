from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg = get_package_share_directory('antbot_navigation')
    ekf_yaml_default = os.path.join(pkg, 'config', 'ekf.yaml')
    ekf_yaml = LaunchConfiguration('ekf_yaml', default=ekf_yaml_default)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('ekf_yaml', default_value=ekf_yaml_default),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',             # ต้องตรงกับคีย์บนสุดของ YAML
            output='screen',
            parameters=[ekf_yaml, {'use_sim_time': use_sim_time}],
        )
    ])
