# Antbot
ที่ใช้
CMD1 เปิด GZ sim, Rviz, spawn Robot, เรียก bridge

source install/setup.bash

ros2 launch antbot_description gz_rviz.launch.py	

CMD2: (แปลง /cmd_vel → TwistStamped)

source ~/antbot_ws/install/setup.bash

ros2 run antbot_drive_controller cmd_vel_to_stamped

CMD3 เรียก efk, robot_localization, ได้ map --> odom --> base_link

source install/setup.bash

ros2 launch antbot_navigation localization.launch.py

CMD4 รัน nav

source install/setup.bash

ros2 launch antbot_navigation nav2.launch.py

---------------------------------------------------

คียบอร์ด

source ~/antbot_ws/install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard

ไว้เก็บแมพ

source install/setup.bash

ros2 launch antbot_navigation slam.launch.py

เซฟแมพ

ros2 run nav2_map_server map_saver_cli -f ~/antbot_ws/src/antbot_maps/maps/my_world

