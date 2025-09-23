cmake_minimum_required(VERSION 3.8)
project(antbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(nav2_planner REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(slam_toolbox REQUIRED)

# ติดตั้งโฟลเดอร์ launch / config / maps ไปไว้ใน share/${PROJECT_NAME}
# (แนะนำให้มีโฟลเดอร์ maps แม้ว่างไว้ เพื่อให้ install ครบ)
install(DIRECTORY
  launch
  include
  config
  maps
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# ติดตั้งสคริปต์ให้ไปอยู่ใน lib/${PROJECT_NAME}
install(PROGRAMS
  scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/scan_frame_fix.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
