# antbot_navigation/launch/nav2.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def find_bt_xml():
    candidates = []
    # ไฟล์ยอดนิยมใน Nav2
    names = [
        "navigate_w_replanning_and_recovery.xml",
        "navigate_w_replanning_time.xml",
    ]
    # ลองทั้ง 2 แพ็กเกจนี้
    for pkg in ("nav2_bt_navigator", "nav2_bringup"):
        try:
            base = get_package_share_directory(pkg)
            for n in names:
                candidates.append(os.path.join(base, "behavior_trees", n))
        except Exception:
            pass

    for p in candidates:
        if os.path.exists(p):
            return p

    raise RuntimeError(
        "Could not find a Nav2 BT XML file. Checked:\n" + "\n".join(candidates)
    )

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')  # คงอาร์กิวเมนต์ไว้เผื่อใช้
    params_file  = LaunchConfiguration('params_file')

    pkg_share = FindPackageShare('antbot_navigation')
    default_params = PathJoinSubstitution([pkg_share, 'config', 'antbot_nav2.yaml'])

    # หา BT XML จริง ๆ ที่ติดตั้งอยู่
    bt_xml = find_bt_xml()

    # หมายเหตุ: ไม่ส่ง {'use_sim_time': use_sim_time} ซ้ำ
    # ให้ตั้งค่าผ่าน YAML (antbot_nav2.yaml) เพื่อเลี่ยง type mismatch
    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    smoother = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[params_file],
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    )
    
    ack_bt_xml = PathJoinSubstitution([
        pkg_share, 'behavior_trees',
        'navigate_w_replanning_and_recovery_ackermann.xml'
    ])

    bt_nav = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            params_file,
            {'default_nav_to_pose_bt_xml': ack_bt_xml},
            {'default_nav_through_poses_bt_xml': ack_bt_xml},
        ],
    )

    waypoint = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
    )

    lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ],
        }],
    )
    cmd_vel_converter = Node(
        package='antbot_drive_controller',
        executable='cmd_vel_to_stamped',
        name='cmd_vel_to_stamped',
        output='screen',
    )


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file',  default_value=default_params),
        controller, planner, smoother, behavior, bt_nav, waypoint, lifecycle, cmd_vel_converter,
    ])