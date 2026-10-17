cmake_minimum_required(VERSION 3.8)
project(sa3_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ============================================================
# Main SA3 motion node
# ============================================================

add_executable(sa3_motion_node
  src/sa3_motion.cpp
)

ament_target_dependencies(sa3_motion_node
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# IK reach test
# ============================================================

add_executable(ik_reach_test
  src/ik_reach_test.cpp
)

ament_target_dependencies(ik_reach_test
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# Add collision cuboid to MoveIt Planning Scene
# ============================================================

add_executable(add_collision_box
  src/add_collision_box.cpp
)

ament_target_dependencies(add_collision_box
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
)

# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

# ============================================================
# Robot1 pick/place motion node
# ============================================================

add_executable(robot1_motion
  src/robot1_motion.cpp
)

target_include_directories(robot1_motion PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot1_motion
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# Robot2 pick/place motion node
# ============================================================

add_executable(robot2_motion
  src/robot2_motion.cpp
)

target_include_directories(robot2_motion PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot2_motion
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# Robot3 pick/place motion node
# ============================================================

add_executable(robot3_motion
  src/robot3_motion.cpp
)

target_include_directories(robot3_motion PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot3_motion
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# Robot4 pick/place motion node
# ============================================================

add_executable(robot4_motion
  src/robot4_motion.cpp
)

target_include_directories(robot4_motion PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot4_motion
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# Robot2 pick/place motion node
# ============================================================

add_executable(robot_1_2_motion
  src/robot_1_2_motion.cpp
)

ament_target_dependencies(robot_1_2_motion
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
  tf2_ros
)


# ============================================================
# Robot1 tying gun
# ============================================================

add_executable(robot1_tying_gun
  src/robot1_tying_gun.cpp
)

ament_target_dependencies(robot1_tying_gun
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# Robot1 tying gun
# ============================================================

add_executable(robot2_tying_gun
  src/robot2_tying_gun.cpp
)

ament_target_dependencies(robot2_tying_gun
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# workspace_scanner
# ============================================================


add_executable(robot1_workspace_scanner
  src/workspace_scanner.cpp
)

ament_target_dependencies(robot1_workspace_scanner
  rclcpp
  geometry_msgs
  visualization_msgs
  moveit_ros_planning_interface
  moveit_core
)

install(TARGETS
  robot1_workspace_scanner
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# home_all_robots
# ============================================================


add_executable(home_all_robots
  src/home_all_robots.cpp
)

ament_target_dependencies(home_all_robots
  rclcpp
  rclcpp_action
  control_msgs
  moveit_ros_planning_interface
  geometry_msgs
  moveit_msgs
  tf2
  tf2_ros
)


# ============================================================
# robot1_gripper_change
# ============================================================

add_executable(robot1_gripper_change
  src/robot1_gripper_change.cpp
)

target_include_directories(robot1_gripper_change PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(robot1_gripper_change
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ============================================================
# YAML-driven MoveIt Planning Scene Objects
# ============================================================

add_library(planning_scene_objects
  src/planning_scene_objects.cpp
)

target_include_directories(planning_scene_objects PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(planning_scene_objects
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
  geometric_shapes
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

target_link_libraries(planning_scene_objects
  yaml-cpp
)

# ============================================================
# Rebar Planning Scene Node
# ============================================================

add_executable(rebar_planning_scene_node
  src/rebar_planning_scene_node.cpp
)

target_link_libraries(rebar_planning_scene_node
  planning_scene_objects
)

ament_target_dependencies(rebar_planning_scene_node
  rclcpp
)

install(
  TARGETS rebar_planning_scene_node
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# Schunk attachment node
# ============================================================

add_executable(attach_tool
  src/attach_tool.cpp
)

ament_target_dependencies(attach_tool
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)



# ============================================================
# Link Planning Scene Objects to robot motion nodes
# ============================================================

target_link_libraries(robot1_motion
  planning_scene_objects
)

target_link_libraries(robot2_motion
  planning_scene_objects
)

target_link_libraries(robot3_motion
  planning_scene_objects
)

target_link_libraries(robot4_motion
  planning_scene_objects
)


# ============================================================
# Install executables
# ============================================================

install(
  TARGETS
    sa3_motion_node
    ik_reach_test
    add_collision_box
    robot1_motion
    robot2_motion
    robot3_motion
    robot4_motion
    robot_1_2_motion
    robot1_tying_gun
    robot2_tying_gun
    robot1_workspace_scanner
    home_all_robots
    robot1_gripper_change
    planning_scene_objects
    attach_tool
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    launch
    config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()