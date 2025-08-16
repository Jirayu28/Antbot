from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    desc_share = get_package_share_directory('antbot_description')
    ctrl_share = get_package_share_directory('antbot_drive_controller')

    world_name = 'empty'
    world_path = os.path.join(desc_share, 'worlds', 'empty.sdf')
    xacro_file = os.path.join(desc_share, 'urdf', 'antbot.urdf.xacro')
    urdf_output_path = os.path.join('/tmp', 'antbot.urdf')
    ctrl_yaml = os.path.join(ctrl_share, 'config', 'tricycle_controller.yaml')

    existing_gz_res = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_res = f"{desc_share}:{ctrl_share}:{existing_gz_res}" if existing_gz_res else f"{desc_share}:{ctrl_share}"

    # ✅ ให้ RSP ปล่อย /robot_description ด้วย QoS ที่ถูกต้อง
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

    # ❌ (ลบทิ้ง) pub_robot_description_once ที่ยิง /robot_description ด้วย ros2 topic pub
    # pub_robot_description_once = ...

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

    return LaunchDescription([
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

        # 3.5) RSP (ให้ CM เห็น /robot_description)
        TimerAction(period=6.0, actions=[rsp_node]),

        # 3.6) ✅ ปลด pause ให้ซิมวิ่งจริงก่อน activate controller
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

        # 5) tricycle (หลังแก้ YAML ข้อ B แล้ว)
        TimerAction(period=9.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'tricycle_controller',
                    '--controller-manager', '/controller_manager',
                    '-p', ctrl_yaml,   # YAML เดียวกัน
                    '--controller-manager-timeout', '30'
                ],
                output='screen'
            )
        ]),
    ])
