cmake_minimum_required(VERSION 3.8)
project(antbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# urdf/xacro
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING
    PATTERN "*.xacro"
    PATTERN "*.urdf"
)

# meshes
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# worlds
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# rviz (ถ้ามี)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

# 🔥 สำคัญ: ติดตั้งไฟล์โมเดลระดับบนสุด
install(FILES model.sdf model.config
  DESTINATION share/${PROJECT_NAME}
)

# (ออปชัน) ติดตั้งสคริปต์ให้รันได้ด้วย ros2 run
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/scripts)
  install(PROGRAMS scripts/laser_frame_fix.py
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
