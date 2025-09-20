# antbot_navigation/launch/slam.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    # default = antbot_navigation/config/mapper_params_online_async.yaml
    default_path_nav = os.path.join(
        get_package_share_directory('antbot_navigation'),
        'config', 'mapper_params_online_async.yaml'
    )

    # ถ้ามีไฟล์ mapper_params_online_async.yaml ก็ให้ผู้ใช้เลือกผ่านอาร์กิวเมนต์ได้
    # (ดีฟอลต์ยังเป็นของ antbot_navigation เพื่อไม่พังถ้าอีกแพ็กเกจไม่มี)
    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    # EKF (ให้เริ่มก่อน)
    ekf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('antbot_navigation'),
                'launch', 'ekf.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('slam_params_file', default_value=default_path_nav),

        # รัน EKF ก่อน
        GroupAction([ekf_include]),

        # เว้นดีเลย์ 1.0 วินาที แล้วค่อยรัน SLAM (กัน “Failed to compute odom pose” ตอนบูต)
        TimerAction(
            period=1.0,
            actions=[GroupAction([slam_include])]
        ),
    ])