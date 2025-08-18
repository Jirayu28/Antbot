from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('antbot_drive_controller')

    mode = LaunchConfiguration('mode')

    # เงื่อนไขที่ถูกต้อง: ใช้ PythonExpression เพื่อให้ IfCondition ได้ Substitution
    keyboard_cond = IfCondition(PythonExpression(['"', mode, '" == "keyboard"']))
    joy_cond      = IfCondition(PythonExpression(['"', mode, '" == "joy"']))

    teleop_kb = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='keyboard_teleop',
        output='screen',
        emulate_tty=True,                         # ป้องกัน termios error
        remappings=[('cmd_vel', '/keyboard/cmd_vel')],
        condition=keyboard_cond
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        condition=joy_cond
    )

    teleop_joy = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_joy',
        output='screen',
        parameters=[PathJoinSubstitution([pkg, 'config', 'teleop_joy.yaml'])],
        remappings=[('cmd_vel', '/joy/cmd_vel')],
        condition=joy_cond
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[PathJoinSubstitution([pkg, 'config', 'twist_mux.yaml'])]
        # ค่า default: ส่งออก /cmd_vel
    )

    adapter = Node(
        package='antbot_drive_controller',
        executable='cmd_vel_to_stamped',
        name='cmd_vel_to_stamped',
        output='screen'
        # แปลง /cmd_vel (Twist) -> /tricycle_controller/cmd_vel (TwistStamped)
    )

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='keyboard', description='keyboard or joy'),
        teleop_kb,
        joy_node,
        teleop_joy,
        twist_mux,
        adapter
    ])
