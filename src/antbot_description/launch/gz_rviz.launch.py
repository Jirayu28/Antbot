# antbot_description/launch/gz_rviz.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node as RosNode
import os

def generate_launch_description():
    desc_share = get_package_share_directory('antbot_description')
    ctrl_share = get_package_share_directory('antbot_drive_controller')

    world_name = 'gmr_world'
    default_world = os.path.join(desc_share, 'worlds', 'gmr_world.sdf')
    world_arg = LaunchConfiguration('world')

    xacro_file = os.path.join(desc_share, 'urdf', 'antbot.urdf.xacro')
    urdf_output_path = os.path.join('/tmp', 'antbot.urdf')
    ctrl_yaml = os.path.join(ctrl_share, 'config', 'tricycle_controller.yaml')
    model_name = 'antbot'

    # Gazebo resource path (ให้หา package นี้แน่นอน)
    existing_gz_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_res = f"{desc_share}:{ctrl_share}:{existing_gz_res}" if existing_gz_res else f"{desc_share}:{ctrl_share}"

    # 1) xacro -> urdf
    xacro_to_urdf = ExecuteProcess(
        cmd=['xacro', xacro_file, f'ctrl_yaml:={ctrl_yaml}', '-o', urdf_output_path],
        output='screen'
    )

    # 2) start Gazebo (server+gui)
    gz_server_gui = ExecuteProcess(
        cmd=['gz', 'sim', world_arg, '-v', '4'],  # จะรับ world จาก argument
        output='screen'
    )

    # 3) spawn model
    spawn_model = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_sim', 'create',
             '-world', world_name,
             '-name', model_name,
             '-x','3','-y','0','-z','0.2',
             '-file', urdf_output_path],
        output='screen'
    )

    # Robot State Publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['cat ', urdf_output_path]), value_type=str),
            'use_sim_time': True
        }]
    )

    # ---------- Bridges ----------
    # Clock
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_clock',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Lidar (GZ -> ROS)
    scan_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_scan',
        arguments=['/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen'
    )

    gz_imu_topic = 'imu'  # ชื่อ GZ ฝั่งระบบ (ไม่มี / นำหน้า)

    imu_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_imu',
        arguments=[f'{gz_imu_topic}@sensor_msgs/msg/Imu[gz.msgs.IMU'],
        remappings=[(gz_imu_topic, '/imu')],   # ROS จะได้ใช้ /imu
        output='screen'
    )
    # Ultrasonic (LaserScan)
    ultra_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_ultrasonic',
        arguments=['/ultrasonic@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'],
        output='screen'
    )


    # Odometry (GZ -> ROS) remap เป็น /odom
    gz_odom_topic = f'/world/{world_name}/model/{model_name}/odometry'
    odom_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='bridge_odom',
        arguments=[f'{gz_odom_topic}@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
        remappings=[(gz_odom_topic, '/odom')],
        output='screen'
    )


    # TF alias (เดิม)
    from launch_ros.actions import Node as RosNode
    lidar_tf_alias = RosNode(
        package='tf2_ros', executable='static_transform_publisher', name='lidar_tf_alias', output='screen',
        arguments=['--frame-id','lidar_sensor_link',
                  '--child-frame-id',f'{model_name}/base_link/lidar_sensor_link',
                  '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0']
    )
    lidar_tf_alias_dummy = RosNode(
        package='tf2_ros', executable='static_transform_publisher', name='lidar_tf_alias_dummy', output='screen',
        arguments=['--frame-id','lidar_sensor_link',
                  '--child-frame-id',f'{model_name}/base_dummy/lidar_sensor_link',
                  '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0']
    )
    ultra_tf_alias = RosNode(
        package='tf2_ros', executable='static_transform_publisher', name='ultrasonic_tf_alias', output='screen',
        arguments=['--frame-id','ultrasonic_front_link',
                  '--child-frame-id',f'{model_name}/base_link/ultrasonic_front_link',
                  '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0']
    )
    ultra_tf_alias_dummy = RosNode(
        package='tf2_ros', executable='static_transform_publisher', name='ultrasonic_tf_alias_dummy', output='screen',
        arguments=['--frame-id','ultrasonic_front_link',
                  '--child-frame-id',f'{model_name}/base_dummy/ultrasonic_front_link',
                  '--x','0','--y','0','--z','0','--roll','0','--pitch','0','--yaw','0']
    )
    imu_tf_alias = RosNode(
    package='tf2_ros', executable='static_transform_publisher',
    name='imu_tf_alias', output='screen',
    arguments=[
        '--frame-id','imu_link',
        '--child-frame-id',f'{model_name}/base_link/imu_link_sensor',
        '--x','0','--y','0','--z','0',
        '--roll','0','--pitch','0','--yaw','0'
    ]
)

    # Unpause physics
    unpause_world = ExecuteProcess(
        cmd=['gz','service','-s',f'/world/{world_name}/control','--reqtype','gz.msgs.WorldControl',
             '--reptype','gz.msgs.Boolean','--timeout','3000','--req','pause:false'],
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Controllers
    cm_ns = '/controller_manager'
    wait_cm = ExecuteProcess(
        cmd=['/bin/bash','-lc',
             f'''echo "[wait_cm] waiting for {cm_ns} ...";
                 until ros2 service list | grep -q "{cm_ns}/list_controllers"; do sleep 0.2; done;
                 echo "[wait_cm] controller_manager is up";'''],
        output='screen'
    )
    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['joint_state_broadcaster','--controller-manager', cm_ns,'--controller-manager-timeout','30','--activate'],
        output='screen'
    )
    spawn_tricycle = Node(
        package='controller_manager', executable='spawner',
        name='spawner_tricycle_controller',
        arguments=['tricycle_controller','--controller-manager', cm_ns,'-p', os.path.join(ctrl_share, 'config', 'tricycle_controller.yaml'),'--controller-manager-timeout','30','--activate'],
        output='screen'
    )
    show_active = ExecuteProcess(
        cmd=['ros2','control','list_controllers','--controller-manager', cm_ns],
        output='screen'
    )
    after_wait_spawn = RegisterEventHandler(OnProcessExit(target_action=wait_cm, on_exit=[spawn_jsb, spawn_tricycle]))
    after_tricycle_print = RegisterEventHandler(OnProcessExit(target_action=spawn_tricycle, on_exit=[show_active]))

    # ---- system plugin paths (IMU + ros2_control) ----
    existing_sys = os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
    plugin_path = ':'.join(p for p in [
        '/opt/ros/jazzy/lib',                                   # ที่อยู่ libgz_ros2_control-system.so
        '/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins',# ที่อยู่ libgz-sim8-imu-system.so (symlink libgz-sim-imu-system.so)
        existing_sys
    ] if p)

    # เผื่อ dynamic linker ต้องใช้
    existing_ld = os.environ.get('LD_LIBRARY_PATH', '')
    ld_path = ':'.join(p for p in [
        '/opt/ros/jazzy/lib',
        existing_ld
    ] if p)


    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=default_world),

        # ให้ GZ หา resource (โมเดล/เวิลด์) ได้
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_res),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=gz_res),

        # สำคัญ: บอกพาธปลั๊กอินระบบ ให้ครอบคลุมทั้ง IMU และ ros2_control + เผื่อของเดิมไว้
        SetEnvironmentVariable(name='GZ_SIM_SYSTEM_PLUGIN_PATH', value=plugin_path),
        SetEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH', value=plugin_path),

        # เผื่อ dynamic linker หา .so ไม่เจอ (บางเครื่องจำเป็น)
        SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=ld_path),

        xacro_to_urdf,
        gz_server_gui,

        TimerAction(period=5.0, actions=[spawn_model]),
        TimerAction(period=6.0, actions=[rsp_node, clock_bridge, scan_bridge, imu_bridge, ultra_bridge, odom_bridge]),
        TimerAction(period=6.2, actions=[lidar_tf_alias, lidar_tf_alias_dummy, ultra_tf_alias, ultra_tf_alias_dummy, imu_tf_alias]),
        TimerAction(period=6.5, actions=[unpause_world]),

        wait_cm, after_wait_spawn, after_tricycle_print,
        TimerAction(period=8.0, actions=[rviz_node]),
    ])
