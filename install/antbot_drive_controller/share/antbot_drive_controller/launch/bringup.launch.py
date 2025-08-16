from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    tricycle_config = os.path.join(
        get_package_share_directory('antbot_drive_controller'),
        'config',
        'tricycle_controller.yaml'
    )

    return LaunchDescription([
        # 1) joint_state_broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'joint_state_broadcaster',
                '--controller-manager', '/controller_manager'
            ],
            output='screen'
        ),

        # 2) tricycle_controller (อ่าน params จากไฟล์)
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[
                'tricycle_controller',
                '--controller-manager', '/controller_manager',
                '--controller-type', 'tricycle_steering_controller/TricycleSteeringController',
                '--param-file', tricycle_config
            ],
            output='screen'
        ),
    ])
