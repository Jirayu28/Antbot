from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='imu_bridge',
            output='screen',
            arguments=['/imu@sensor_msgs/msg/Imu@gz.msgs.IMU]'],
        )
    ])