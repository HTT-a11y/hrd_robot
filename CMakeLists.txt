cmake_minimum_required(VERSION 3.22)
project(hrd_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(PROGRAMS
  scripts/demo_dual_arm_planning.py
  scripts/demo_dual_arm_reach.py
  scripts/spawn_target_object.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
