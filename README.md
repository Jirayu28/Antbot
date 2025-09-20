# antbotv1

CMD1 
source install/setup.bash
ros2 launch antbot_description gz_rviz.launch.py

CMD2
source ~/antbot_ws/install/setup.bash
ros2 run antbot_drive_controller cmd_vel_to_stamped

CMD3
source install/setup.bash
ros2 launch antbot_navigation localization.launch.py

CMD4
source install/setup.bash
ros2 launch antbot_navigation nav2.launch.py
