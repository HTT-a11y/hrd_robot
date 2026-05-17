# Install script for directory: /root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/install/hrd_moveit_config")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config/demo_dual_arm_reach_cpp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config/demo_dual_arm_reach_cpp")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config/demo_dual_arm_reach_cpp"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config" TYPE EXECUTABLE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/demo_dual_arm_reach_cpp")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config/demo_dual_arm_reach_cpp" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config/demo_dual_arm_reach_cpp")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config/demo_dual_arm_reach_cpp"
         OLD_RPATH "/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config/demo_dual_arm_reach_cpp")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/hrd_moveit_config" TYPE PROGRAM FILES
    "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/scripts/demo_dual_arm_planning.py"
    "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/scripts/demo_dual_arm_reach.py"
    "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/scripts/spawn_target_object.py"
    "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/scripts/print_tcp_pose.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE DIRECTORY FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/launch" REGEX "/setup\\_assistant\\.launch$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE DIRECTORY FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/.setup_assistant")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/hrd_moveit_config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/hrd_moveit_config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config/environment" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config/environment" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_index/share/ament_index/resource_index/packages/hrd_moveit_config")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config/cmake" TYPE FILE FILES
    "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_core/hrd_moveit_configConfig.cmake"
    "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/ament_cmake_core/hrd_moveit_configConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hrd_moveit_config" TYPE FILE FILES "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/root/HRD_ws/hrd_project/hrd_ros2/src/hrd_moveit_config/build/hrd_moveit_config/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
