cmake_minimum_required(VERSION 3.8)
project(antbot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 🔍 Find dependencies
find_package(ament_cmake REQUIRED)

# 📦 launch
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 🧩 urdf/xacro (รองรับทั้ง .xacro และ .urdf ถ้ามี)
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
  FILES_MATCHING
    PATTERN "*.xacro"
    PATTERN "*.urdf"
)

# 🧱 meshes (ก็อปทั้งหมด)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# 🌍 worlds (ก็อปทั้งโฟลเดอร์ เพื่อรองรับ asset อื่น ๆ)
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

# 🧭 rviz config (ติดตั้งเฉพาะถ้ามีโฟลเดอร์ rviz)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/rviz)
  install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}/rviz
  )
endif()

# ✅ tests (ถ้ามี)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
