cmake_minimum_required(VERSION 3.22)
project(hrd_moveit_config)

# ── C++ build deps ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# ── C++ demo executable ────────────────────────────────────────────────────
add_executable(demo_dual_arm_reach_cpp src/demo_dual_arm_reach.cpp)
ament_target_dependencies(demo_dual_arm_reach_cpp
  rclcpp
  rclcpp_action
  moveit_msgs
  geometry_msgs
  shape_msgs)
install(TARGETS demo_dual_arm_reach_cpp DESTINATION lib/${PROJECT_NAME})

# ── Python scripts ─────────────────────────────────────────────────────────
install(PROGRAMS
  scripts/demo_dual_arm_planning.py
  scripts/demo_dual_arm_reach.py
  scripts/spawn_target_object.py
  scripts/print_tcp_pose.py
  DESTINATION lib/${PROJECT_NAME})

# ── Config & launch files ──────────────────────────────────────────────────
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

ament_package()
