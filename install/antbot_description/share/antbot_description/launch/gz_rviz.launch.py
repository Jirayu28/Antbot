# antbot_description/launch/gz_rviz.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # ====== ส่วน Gazebo (คงโค้ดเดิม) ======
    desc_share = get_package_share_directory('antbot_description')
    ctrl_share = get_package_share_directory('antbot_drive_controller')

    world_name = 'empty'
    world_path = os.path.join(desc_share, 'worlds', 'empty.sdf')
    xacro_file = os.path.join(desc_share, 'urdf', 'antbot.urdf.xacro')
    urdf_output_path = os.path.join('/tmp', 'antbot.urdf')
    ctrl_yaml = os.path.join(ctrl_share, 'config', 'tricycle_controller.yaml')

    existing_gz_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_res = f"{desc_share}:{ctrl_share}:{existing_gz_res}" if existing_gz_res else f"{desc_share}:{ctrl_share}"

    # ✅ ให้ RSP ปล่อย /robot_description + ใช้ sim time
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', xacro_file]), value_type=str),
            'use_sim_time': True
        }]
    )

    # ✅ Bridges (เพิ่ม /clock และ /scan)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen'
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
        output='screen'
    )

    # ✅ คำสั่ง unpause world: pause:false
    unpause_world = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', f'/world/{world_name}/control',
            '--reqtype', 'gz.msgs.WorldControl',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000',
            '--req', 'pause:false'
        ],
        output='screen'
    )

    # ====== ส่วน RViz / GUI (คงโค้ดเดิมจาก display.launch.py โดย "ไม่" สร้าง RSP ซ้ำ) ======
    robot_description_param = ParameterValue(
        Command([
            'xacro ',  # ← ✅ ต้องมีเว้นวรรค
            PathJoinSubstitution([
                FindPackageShare('antbot_description'),
                'urdf',
                'antbot.urdf.xacro'
            ])
        ]),
        value_type=str
    )
    # หมายเหตุ: ไม่ได้ส่ง param นี้ให้กับโหนดใด ๆ เพราะ RSP ตัวหลักสร้างจากฝั่ง Gazebo อยู่แล้ว

    jsp_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],  # ✅ ให้ RViz ตามเวลาในซิม
    )

    # ====== ประกอบ LaunchDescription ======
    return LaunchDescription([
        # --- Env สำหรับ Gazebo ---
        SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=gz_res),
        SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH', value=gz_res),

        # 1) xacro -> urdf
        ExecuteProcess(cmd=['xacro', xacro_file, '-o', urdf_output_path], output='screen'),

        # 2) start GZ
        ExecuteProcess(cmd=['gz', 'sim', world_path, '-v', '4'], output='screen'),

        # 3) spawn model
        TimerAction(period=5.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'ros_gz_sim', 'create',
                     '-world', world_name,
                     '-name', 'antbot',
                     '-z', '0.2',
                     '-file', urdf_output_path],
                output='screen'
            )
        ]),

        # 3.5) RSP + bridges
        TimerAction(period=6.0, actions=[rsp_node, clock_bridge, scan_bridge]),

        # 3.6) unpause
        TimerAction(period=6.5, actions=[unpause_world]),

        # 4) JSB
        TimerAction(period=7.5, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    '--controller-manager-timeout', '30'
                ],
                output='screen'
            )
        ]),

        # 5) tricycle
        TimerAction(period=9.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'tricycle_controller',
                    '--controller-manager', '/controller_manager',
                    '-p', ctrl_yaml,
                    '--controller-manager-timeout', '30'
                ],
                output='screen'
            )
        ]),

        # 6) GUI
        jsp_gui_node,
        rviz_node,
    ])
