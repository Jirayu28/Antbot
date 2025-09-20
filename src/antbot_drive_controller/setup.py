# antbot_drive_controller/setup.py
from setuptools import setup

package_name = 'antbot_drive_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # ต้องมี antbot_drive_controller/__init__.py
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ✅ ติดตั้งไฟล์ config ที่ใช้จริง รวม mapper_params_online_async.yaml
        ('share/' + package_name + '/config', [
            'config/tricycle_controller.yaml',
            'config/teleop_joy.yaml',
            'config/twist_mux.yaml',
            
        ]),
        # ✅ ติดตั้ง launch ครบ รวม slam.launch.py
        ('share/' + package_name + '/launch', [
            'launch/teleop_all.launch.py',
            'launch/slam.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jirayu',
    maintainer_email='jirayu.pkd@gmail.com',
    description='Teleop + mux + Twist->TwistStamped + LaserScan->Range + IMU relay for Antbot',
    entry_points={
        'console_scripts': [
            'cmd_vel_to_stamped = antbot_drive_controller.cmd_vel_to_stamped:main',
            'scan_to_range      = antbot_drive_controller.scan_to_range:main',
            # ✅ เพิ่ม IMU relay (แก้ frame_id + covariance)
            'imu_frame_relay    = antbot_drive_controller.imu_frame_relay:main',
        ],
    },
)
