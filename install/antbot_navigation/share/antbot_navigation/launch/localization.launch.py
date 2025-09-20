from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml     = LaunchConfiguration('map')
    params_file  = LaunchConfiguration('params_file')

    auto_initial_pose = LaunchConfiguration('auto_initial_pose')
    init_x     = LaunchConfiguration('init_x')
    init_y     = LaunchConfiguration('init_y')
    init_yaw   = LaunchConfiguration('init_yaw_deg')
    init_delay = LaunchConfiguration('init_delay_sec')

    nav_share  = FindPackageShare('antbot_navigation')

    default_map    = PathJoinSubstitution([nav_share, 'maps',   'my_world.yaml'])
    default_params = PathJoinSubstitution([nav_share, 'config', 'antbot_nav2.yaml'])
    default_ekf    = PathJoinSubstitution([nav_share, 'config', 'ekf.yaml'])

    # --- EKF include ---
    ekf_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([nav_share, 'launch', 'ekf.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time, 'ekf_yaml': default_ekf}.items()
    )

    # --- Nav2 localization ---
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]
    )
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # --- Auto initial pose ---
    init_pose_node = Node(
        package='antbot_navigation', executable='publish_initial_pose.py',
        name='publish_initial_pose', output='screen',
        condition=IfCondition(auto_initial_pose),
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': 'map',
            'x': init_x, 'y': init_y, 'yaw_deg': init_yaw,
            'wait_for_subscribers_sec': 3.0,
            'extra_delay_sec': 1.0,
            'backstamp_sec': 0.5,
            'amcl_transform_tolerance': 0.25,
        }]
    )

    # --- Scheduling ---
    delayed_core      = TimerAction(period=2.0, actions=[GroupAction([ekf_include, map_server, amcl])])
    delayed_lifecycle = TimerAction(period=3.0, actions=[lifecycle_manager])
    delayed_init_pose = TimerAction(period=init_delay, actions=[init_pose_node])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',      default_value='true'),
        DeclareLaunchArgument('map',               default_value=default_map),
        DeclareLaunchArgument('params_file',       default_value=default_params),
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        DeclareLaunchArgument('init_x',            default_value='0.0'),
        DeclareLaunchArgument('init_y',            default_value='0.0'),
        DeclareLaunchArgument('init_yaw_deg',      default_value='0.0'),
        DeclareLaunchArgument('init_delay_sec',    default_value='4.0'),

        delayed_core, delayed_lifecycle, delayed_init_pose,
    ])
