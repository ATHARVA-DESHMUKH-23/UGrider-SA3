cmake_minimum_required(VERSION 3.8)
project(sa2_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY
    urdf
    meshes
    launch
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/workcell_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(environment_collision
  src/environment_collision.cpp
)

ament_target_dependencies(environment_collision
  rclcpp
  moveit_msgs
  urdf
  std_msgs
  moveit_ros_planning_interface
  geometric_shapes
  geometry_msgs
  shape_msgs
)

install(
  TARGETS
    environment_collision
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(environment_collision
  rclcpp
  moveit_msgs
  geometry_msgs
)


install(
  PROGRAMS
    scripts/workcell_visualizer.py
    scripts/arc_frame_base.py
    scripts/rebar_visualizer.py
    scripts/gripper_visualizer.py
    scripts/movable_rebar_tf.py
    scripts/movable_rebar_marker.py
  DESTINATION lib/${PROJECT_NAME}
)


install(
  TARGETS
    environment_collision
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
