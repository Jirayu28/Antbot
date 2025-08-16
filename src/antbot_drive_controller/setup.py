from setuptools import setup

package_name = 'antbot_drive_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # non-src layout: ต้องมีโฟลเดอร์ antbot_drive_controller พร้อม __init__.py
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/tricycle_controller.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/bringup.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pakorn',
    maintainer_email='titlepakornza@gmail.com',
    description='Teleop + controller bringup for AntBot',
    entry_points={
        'console_scripts': [
            'teleop_keyboard = antbot_drive_controller.teleop_keyboard:main',
            'cmd_vel_to_stamped = antbot_drive_controller.cmd_vel_to_stamped:main',
        ],
    },
)
